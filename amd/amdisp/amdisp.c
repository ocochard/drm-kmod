/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Olivier Cochard <olivier@FreeBSD.org>
 */

/*
 * amdisp: test driver for the AMD ISP 4.1 camera block (Strix Halo), on
 * top of the services amdgpu exports (amdgpu_isp_freebsd.h).  The register
 * and command sequences were measured on Linux, not taken from its code.
 *
 * sysctl hw.amdisp.boot=1 powers the ISP up and boots its firmware (=0
 * stops it); hw.amdisp.cmd=1 sends the memory pool, =2 sets up a stream
 * and waits for one frame.  AMD GPIO 85 must be high first (gpioctl).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/errno.h>
#include <machine/atomic.h>

#include <sys/bus.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>

#include <dev/video/video.h>

#include "amdgpu_isp_freebsd.h"
#include "video_if.h"
#include "gpio_if.h"

#include <sys/gpio.h>

#define	ISP_RD(off)		amdgpu_isp_fbsd_rd(off)
#define	ISP_WR(off, v)		amdgpu_isp_fbsd_wr((off), (v))
#define	LO32(x)			((uint32_t)(x))
#define	HI32(x)			((uint32_t)((uint64_t)(x) >> 32))
#define	MDELAY(ms)		DELAY((ms) * 1000)

#define	ISP_TILES		0x60000
#define	ISP_CCPU_RUN		0x62000	/* bit 0: processor held in reset */
#define	ISP_CCPU_CTRL		0x62054	/* bit 18: processor halted */
#define	ISP_BOOT_STATUS		0x62058
#define	ISP_INT_ENABLE		0x62010

#define	ISP_RING_SLOT		0xaaa80	/* (16 MB - 1) / 24, as on Linux */
#define	ISP_RING_SIZE		0xa00
#define	ISP_LOG_RING		0x62148
#define	ISP_LOG_SIZE		0x200000
#define	ISP_BOOT_READY		0x2402
#define	ISP_INT_STATUS		0x62014
#define	ISP_INT_ACK		0x62018

/* Command ring: slot 0, pointers at +0xc (read) and +0x10 (write). */
#define	ISP_CMD_RING		0x62170
#define	ISP_CMD_RPTR		(ISP_CMD_RING + 0xc)
#define	ISP_CMD_WPTR		(ISP_CMD_RING + 0x10)
#define	ISP_CMD_SLOT		0
/* Response ring: slot 8. */
#define	ISP_RESP_RING		0x62210
#define	ISP_RESP_RPTR		(ISP_RESP_RING + 0xc)
#define	ISP_RESP_WPTR		(ISP_RESP_RING + 0x10)
#define	ISP_RESP_SLOT		8

#define	ISP_ENTRY_SIZE		64	/* command and response entries */
#define	ISP_POOL_SIZE		(100 * 1024 * 1024)

#define	ISP_CMD_SET_STREAM_CONFIG 0x02010001
#define	ISP_CMD_SET_OUT_CHAN_PROP 0x02010003
#define	ISP_CMD_ENABLE_OUT_CHAN	0x02010005
#define	ISP_CMD_START_STREAM	0x02010007
#define	ISP_CMD_STOP_STREAM	0x02010008
#define	ISP_CMD_SEND_BUFFER	0x02040001
#define	ISP_BUF_TYPE_POOL	0xf
#define	ISP_BUF_TYPE_META	0xa
#define	ISP_BUF_TYPE_FRAME	0x8
#define	ISP_RESP_CMD_DONE	0x00010001
#define	ISP_RESP_FRAME_DONE	0x00030001

#define	ISP_NMETA		8
#define	ISP_META_SIZE		0x8000
#define	ISP_NFRAME		4
#define	ISP_WIDTH		1920
#define	ISP_HEIGHT		1080
#define	ISP_Y_SIZE		(ISP_WIDTH * ISP_HEIGHT)
#define	ISP_FRAME_SIZE		(ISP_Y_SIZE * 3 / 2)	/* NV12 */
#define	ISP_YUYV_SIZE		(ISP_Y_SIZE * 2)	/* YUYV, one plane */
#define	ISP_FRAME_ALLOC		ISP_YUYV_SIZE		/* fits both formats */
/* Firmware image formats (SET_OUT_CHAN_PROP word 1). */
#define	ISP_FW_FMT_NV12		1
#define	ISP_FW_FMT_YUYV		7	/* YUV 4:2:2 interleaved */

/* Ring base register (low address) and its slot in the ring buffer. */
static const struct {
	uint32_t	reg;
	uint32_t	slot;
} isp_rings[] = {
	{ 0x62170, 0 },		/* command ring, stream 1 */
	{ 0x62184, 1 },
	{ 0x62198, 2 },
	{ 0x621ac, 3 },
	{ 0x62210, 8 },		/* response ring, stream 1 */
	{ 0x62224, 9 },
	{ 0x62238, 10 },
	{ 0x6224c, 11 },
};

static void *isp_ring_bo, *isp_log_bo, *isp_pool_bo;
static uint64_t isp_ring_gpu, isp_log_gpu, isp_pool_gpu;
static void *isp_ring_cpu, *isp_log_cpu, *isp_pool_cpu;
static int isp_booted;
static uint32_t isp_cmd_seq;
static void *isp_meta_bo[ISP_NMETA], *isp_frame_bo[ISP_NFRAME];
static uint64_t isp_meta_gpu[ISP_NMETA], isp_frame_gpu[ISP_NFRAME];
static void *isp_meta_cpu[ISP_NMETA], *isp_frame_cpu[ISP_NFRAME];
static int isp_last_frame = -1;
static int isp_nframes, isp_requeue;
static struct video_device *isp_vd;
static uint32_t isp_video_seq;
static int isp_yuyv;		/* current format: 0 NV12, 1 YUYV */
#define	ISP_CUR_SIZE	(isp_yuyv ? ISP_YUYV_SIZE : ISP_FRAME_SIZE)
static sbintime_t isp_last_done;
static uint64_t isp_gap_us, isp_copy_us;
static u_int isp_timed, isp_nobuf;

static SYSCTL_NODE(_hw, OID_AUTO, amdisp, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "AMD ISP 4.1 camera test");

static void
isp_ring_set(uint32_t reg, uint64_t gpu,
    uint32_t size)
{
	ISP_WR(reg + 0xc, 0);		/* read pointer */
	ISP_WR(reg + 0x10, 0);		/* write pointer */
	ISP_WR(reg, LO32(gpu));
	ISP_WR(reg + 0x4, HI32(gpu));
	ISP_WR(reg + 0x8, size);
}

static void
isp_smu(int req, uint32_t arg)
{
	int r;

	r = amdgpu_isp_fbsd_smu(req, arg);
	if (r != 0)
		printf("amdisp: SMU request %d failed: %d\n", req, r);
}

static void
isp_free(void)
{
	amdgpu_isp_fbsd_free(&isp_ring_bo, &isp_ring_gpu, &isp_ring_cpu);
	amdgpu_isp_fbsd_free(&isp_log_bo, &isp_log_gpu, &isp_log_cpu);
	amdgpu_isp_fbsd_free(&isp_pool_bo, &isp_pool_gpu, &isp_pool_cpu);
	for (int i = 0; i < ISP_NMETA; i++)
		amdgpu_isp_fbsd_free(&isp_meta_bo[i], &isp_meta_gpu[i],
		    &isp_meta_cpu[i]);
	for (int i = 0; i < ISP_NFRAME; i++)
		amdgpu_isp_fbsd_free(&isp_frame_bo[i], &isp_frame_gpu[i],
		    &isp_frame_cpu[i]);
	isp_last_frame = -1;
}

/*
 * A command or response entry is 16 32-bit words: sequence number,
 * identifier, 12 payload words, a zero word and a checksum, the 32-bit sum
 * of the first 15 words.
 */
static int
isp_send_cmd(uint32_t id, const uint32_t *payload,
    int nwords)
{
	uint32_t e[ISP_ENTRY_SIZE / 4], rptr, wptr, sum;
	uint8_t *ring;
	int i;

	if (nwords > 12)
		return (EINVAL);
	memset(e, 0, sizeof(e));
	e[0] = ++isp_cmd_seq;
	e[1] = id;
	for (i = 0; i < nwords; i++)
		e[2 + i] = payload[i];
	for (sum = 0, i = 0; i < 15; i++)
		sum += e[i];
	e[15] = sum;

	rptr = ISP_RD(ISP_CMD_RPTR);
	wptr = ISP_RD(ISP_CMD_WPTR);
	if ((wptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE == rptr)
		return (ENOSPC);
	ring = (uint8_t *)isp_ring_cpu + ISP_CMD_SLOT * ISP_RING_SLOT;
	memcpy(ring + wptr, e, sizeof(e));
	wmb();
	ISP_WR(ISP_CMD_WPTR, (wptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE);
	if (!isp_requeue)
		printf("amdisp: sent cmd 0x%08x seq %u at ring offset 0x%x\n",
		    id, e[0], wptr);
	return (0);
}

static void isp_frame_done(uint64_t meta_gpu);
static void isp_dump_regs(const char *when);
static int isp_send_pool(void);

/* Poll the response ring for up to timeout_ms, printing each response. */
static int
isp_poll_resp_type(int timeout_ms, uint32_t want)
{
	uint32_t e[ISP_ENTRY_SIZE / 4], rptr, wptr, st, sum;
	uint8_t *ring;
	int i, n, t;

	ring = (uint8_t *)isp_ring_cpu + ISP_RESP_SLOT * ISP_RING_SLOT;
	for (n = 0, t = 0; t < timeout_ms; t++) {
		st = ISP_RD(ISP_INT_STATUS);
		if (st != 0) {
			ISP_WR(ISP_INT_ACK, st);
			if (!isp_requeue)
				printf("amdisp: interrupt status 0x%x at %d ms\n", st,
			    t);
		}
		rptr = ISP_RD(ISP_RESP_RPTR);
		wptr = ISP_RD(ISP_RESP_WPTR);
		while (rptr != wptr) {
			rmb();
			memcpy(e, ring + rptr, sizeof(e));
			for (sum = 0, i = 0; i < 15; i++)
				sum += e[i];
			if (!isp_requeue)
			printf("amdisp: resp seq %u type 0x%08x w2 0x%x w3 0x%x "
			    "w4 0x%x w5 0x%x checksum %s at %d ms\n", e[0], e[1],
			    e[2], e[3], e[4], e[5], sum == e[15] ? "ok" : "BAD",
			    t);
			rptr = (rptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE;
			ISP_WR(ISP_RESP_RPTR, rptr);
			if (e[1] == ISP_RESP_FRAME_DONE)
				isp_frame_done((uint64_t)e[3] << 32 | e[2]);
			if (want == 0 || e[1] == want)
				n++;
		}
		if (n > 0)
			return (0);
		MDELAY(1);
	}
	printf("amdisp: no response after %d ms (cmd rptr 0x%x wptr 0x%x)\n",
	    timeout_ms, ISP_RD(ISP_CMD_RPTR), ISP_RD(ISP_CMD_WPTR));
	return (ETIMEDOUT);
}

/* Read the responses already in the ring, without waiting. */
static void
isp_service(void)
{
	uint32_t e[ISP_ENTRY_SIZE / 4], rptr, wptr, st;
	uint8_t *ring;

	ring = (uint8_t *)isp_ring_cpu + ISP_RESP_SLOT * ISP_RING_SLOT;
	st = ISP_RD(ISP_INT_STATUS);
	if (st != 0)
		ISP_WR(ISP_INT_ACK, st);
	rptr = ISP_RD(ISP_RESP_RPTR);
	wptr = ISP_RD(ISP_RESP_WPTR);
	while (rptr != wptr) {
		rmb();
		memcpy(e, ring + rptr, sizeof(e));
		rptr = (rptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE;
		ISP_WR(ISP_RESP_RPTR, rptr);
		if (e[1] == ISP_RESP_FRAME_DONE)
			isp_frame_done((uint64_t)e[3] << 32 | e[2]);
	}
}

static int
isp_poll_resp(int timeout_ms)
{
	return (isp_poll_resp_type(timeout_ms, 0));
}

/* Send a command and wait for its "command done". */
static int
isp_cmd_wait(uint32_t id, const uint32_t *p,
    int n)
{
	int r;

	r = isp_send_cmd(id, p, n);
	if (r == 0)
		r = isp_poll_resp_type(2000, ISP_RESP_CMD_DONE);
	return (r);
}

/* Give a meta info or frame buffer back to the firmware (no wait). */
static void
isp_requeue_buf(uint32_t type, uint64_t gpu, uint32_t size)
{
	uint32_t p[9];
	uint64_t uv;

	memset(p, 0, sizeof(p));
	p[0] = type;
	p[2] = 4;
	p[3] = LO32(gpu);
	p[4] = HI32(gpu);
	if (type == ISP_BUF_TYPE_FRAME && isp_yuyv) {
		p[5] = ISP_YUYV_SIZE;
		isp_send_cmd(ISP_CMD_SEND_BUFFER, p, 9);
	} else if (type == ISP_BUF_TYPE_FRAME) {
		uv = gpu + ISP_Y_SIZE;
		p[5] = ISP_Y_SIZE;
		p[6] = LO32(uv);
		p[7] = HI32(uv);
		p[8] = ISP_Y_SIZE / 2;
		isp_send_cmd(ISP_CMD_SEND_BUFFER, p, 9);
	} else {
		p[5] = size;
		isp_send_cmd(ISP_CMD_SEND_BUFFER, p, 6);
	}
}

/* A finished frame: the meta info tells which frame buffer it is. */
static void
isp_frame_done(uint64_t meta_gpu)
{
	uint32_t *m, status, w, h;
	uint64_t y;
	uint8_t *f;
	uint64_t sum;
	int i, j, k;

	for (i = 0; i < ISP_NMETA; i++)
		if (isp_meta_gpu[i] == meta_gpu)
			break;
	if (i == ISP_NMETA) {
		printf("amdisp: frame done with unknown meta 0x%llx\n",
		    (unsigned long long)meta_gpu);
		return;
	}
	m = isp_meta_cpu[i];
	isp_nframes++;
	status = m[0x14 / 4];
	w = m[0x34 / 4];
	h = m[0x38 / 4];
	y = (uint64_t)m[0x54 / 4] << 32 | m[0x50 / 4];
	for (j = 0; j < ISP_NFRAME; j++)
		if (isp_frame_bo[j] != NULL && isp_frame_gpu[j] == y)
			break;
	if (j == ISP_NFRAME) {
		/* Meta info came back without a frame (no buffer queued). */
		if (isp_requeue)
			isp_requeue_buf(ISP_BUF_TYPE_META, isp_meta_gpu[i],
			    ISP_META_SIZE);
		return;
	}
	f = isp_frame_cpu[j];
	for (sum = 0, k = 0; k < ISP_Y_SIZE; k += 4093)
		sum += f[k];
	if (isp_nframes <= 4 || isp_nframes % 10 == 0)
		printf("amdisp: frame %u status %u %ux%u buffer %d mean Y "
		    "%llu\n", m[0], status, w, h, j,
		    (unsigned long long)(sum / (ISP_Y_SIZE / 4093 + 1)));
	isp_last_frame = j;
	if (isp_vd != NULL && status == 3) {
		struct video_buf *vb;
		sbintime_t t0, now;

		/* XXX timing debug: frame interval and copy time. */
		t0 = sbinuptime();
		if (isp_last_done != 0)
			isp_gap_us += (t0 - isp_last_done) / SBT_1US;
		isp_last_done = t0;
		vb = video_buf_acquire(isp_vd);
		if (vb == NULL)
			isp_nobuf++;
		if (vb != NULL) {
			if (video_buf_write(vb, 0, f, ISP_CUR_SIZE) != 0)
				video_buf_error(vb);
			else
				video_buf_done(vb, ISP_CUR_SIZE,
				    isp_video_seq++);
		}
		now = sbinuptime();
		isp_copy_us += (now - t0) / SBT_1US;
		if (isp_video_seq == 90)
			isp_dump_regs("while streaming (frame 90)");
		if (++isp_timed == 60) {
			printf("amdisp: 60 frames: interval %ju us, copy %ju us, "
			    "no buffer %u\n", (uintmax_t)isp_gap_us / 59,
			    (uintmax_t)isp_copy_us / 60, isp_nobuf);
			isp_timed = 0;
			isp_gap_us = isp_copy_us = 0;
			isp_last_done = 0;
			isp_nobuf = 0;
		}
	}
	if (isp_requeue) {
		isp_requeue_buf(ISP_BUF_TYPE_META, isp_meta_gpu[i],
		    ISP_META_SIZE);
		isp_requeue_buf(ISP_BUF_TYPE_FRAME, isp_frame_gpu[j], 0);
	}
}

/* Print the non-zero ISP control, GPIO, I2C and PHY registers. */
static void
isp_dump_regs(const char *when)
{
	static const uint32_t r[][2] = {
		{ 0x60000, 0x60004 }, { 0x62000, 0x62060 },
		{ 0x66000, 0x66800 },
	};
	uint32_t off, v;
	int i;

	printf("amdisp: registers %s\n", when);
	for (i = 0; i < nitems(r); i++)
		for (off = r[i][0]; off < r[i][1]; off += 4) {
			v = ISP_RD(off);
			if (v != 0 && v != 0xffffffff)
				printf("amdisp: reg 0x%05x 0x%08x\n", off, v);
		}
}

/*
 * Sample the ISP's first I2C controller (DesignWare, at 0x66400) every
 * 100 us and print each change: target address (IC_TAR, +0x04), status
 * (IC_STATUS, +0x70), raw interrupts (+0x34), TX/RX FIFO levels (+0x74,
 * +0x78), abort source (+0x80); plus the first MIPI PHY register.
 */
static void
isp_watch_i2c(int ms)
{
	static const uint32_t regs[] = { 0x66404, 0x66470, 0x66434, 0x66474,
	    0x66478, 0x66480, 0x66700 };
	uint32_t prev[nitems(regs)], v[nitems(regs)];
	int i, t, changed, n;

	memset(prev, 0xff, sizeof(prev));
	for (n = 0, t = 0; t < ms * 10 && n < 300; t++) {
		changed = 0;
		for (i = 0; i < nitems(regs); i++)
			v[i] = ISP_RD(regs[i]);
		/* Report target, abort and PHY changes, and busy/idle edges. */
		if (v[0] != prev[0] || v[5] != prev[5] || v[6] != prev[6] ||
		    (v[1] & 0x20) != (prev[1] & 0x20))
			changed = 1;
		if (changed) {
			printf("amdisp: i2c %4d.%d ms tar 0x%x st 0x%x raw 0x%x "
			    "txfl %u rxfl %u abrt 0x%x phy 0x%x\n", t / 10,
			    t % 10, v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
			memcpy(prev, v, sizeof(prev));
			n++;
		}
		DELAY(100);
	}
}

/*
 * Same registers as isp_watch_i2c(), sampled in a tight loop and recorded
 * in memory (printing would slow the loop): every change of target,
 * status, raw interrupts or abort source, with the time in microseconds.
 */
#define	ISP_WATCH_MAX	2000
static struct {
	uint32_t	us, tar, st, raw, abrt, txfl;
} isp_ev[ISP_WATCH_MAX];

static void
isp_watch_fast(int ms)
{
	sbintime_t t0, end;
	uint32_t tar, st, raw, abrt, txfl, p[4];
	int i, n;

	memset(p, 0xff, sizeof(p));
	t0 = sbinuptime();
	end = t0 + ms * SBT_1MS;
	for (n = 0; sbinuptime() < end && n < ISP_WATCH_MAX; ) {
		tar = ISP_RD(0x66404);
		st = ISP_RD(0x66470);
		raw = ISP_RD(0x66434);
		abrt = ISP_RD(0x66480);
		if (tar == p[0] && st == p[1] && raw == p[2] && abrt == p[3])
			continue;
		txfl = ISP_RD(0x66474);
		isp_ev[n].us = (uint32_t)((sbinuptime() - t0) / SBT_1US);
		isp_ev[n].tar = tar;
		isp_ev[n].st = st;
		isp_ev[n].raw = raw;
		isp_ev[n].abrt = abrt;
		isp_ev[n].txfl = txfl;
		p[0] = tar; p[1] = st; p[2] = raw; p[3] = abrt;
		n++;
	}
	printf("amdisp: fast watch: %d changes\n", n);
	/* Print target and abort changes, and the first 60 events. */
	for (i = 0; i < n; i++)
		if (i < 60 || (i > 0 && (isp_ev[i].tar != isp_ev[i - 1].tar ||
		    isp_ev[i].abrt != isp_ev[i - 1].abrt)))
			printf("amdisp: f %6u us tar 0x%x st 0x%x raw 0x%x "
			    "abrt 0x%x txfl %u\n", isp_ev[i].us, isp_ev[i].tar,
			    isp_ev[i].st, isp_ev[i].raw, isp_ev[i].abrt,
			    isp_ev[i].txfl);
}

/*
 * Stream set-up as measured on Linux, then wait for one frame: stream
 * configuration, NV12 1920x1080 on the preview channel, eight meta info
 * buffers, four frame buffers, start.
 */
static int
isp_stream_setup(void)
{
	uint32_t cfg[5] = { 1, 0, 0x5f91, 1, 0 };
	uint32_t prop[7] = { 0, ISP_FW_FMT_NV12, ISP_WIDTH, ISP_HEIGHT,
	    ISP_WIDTH, ISP_WIDTH, 0 };
	uint32_t en[2] = { 0, 1 };
	uint32_t p[9];
	uint64_t uv;
	int i, r;

	/*
	 * Sensor GPIO (amdgpu's ISP411_GPIO_SENSOR0_OFFSET): 0x11 on Linux
	 * while streaming, 0x01 here without this write.  Bit 4 looks like the
	 * output level that switches the sensor on.
	 */
	printf("amdisp: sensor GPIO 0x6613c was 0x%x\n", ISP_RD(0x6613c));
	ISP_WR(0x6613c, ISP_RD(0x6613c) | 0x10);
	MDELAY(10);
	if (isp_yuyv) {
		prop[1] = ISP_FW_FMT_YUYV;
		prop[4] = ISP_WIDTH * 2;	/* luma pitch */
		prop[5] = 0;			/* no chroma plane */
	}
	if ((r = isp_send_pool()) != 0)
		return (r);
	if ((r = isp_send_cmd(ISP_CMD_SET_STREAM_CONFIG, cfg,
	    nitems(cfg))) != 0)
		return (r);
	if ((r = isp_poll_resp_type(2000, ISP_RESP_CMD_DONE)) != 0 ||
	    (r = isp_cmd_wait(ISP_CMD_SET_OUT_CHAN_PROP, prop,
	    nitems(prop))) != 0 ||
	    (r = isp_cmd_wait(ISP_CMD_ENABLE_OUT_CHAN, en,
	    nitems(en))) != 0)
		return (r);

	for (i = 0; i < ISP_NMETA; i++) {
		if (isp_meta_bo[i] == NULL &&
		    (r = amdgpu_isp_fbsd_alloc(ISP_META_SIZE, &isp_meta_bo[i],
		    &isp_meta_gpu[i], &isp_meta_cpu[i])) != 0)
			return (r);
		memset(isp_meta_cpu[i], 0, ISP_META_SIZE);
		memset(p, 0, sizeof(p));
		p[0] = ISP_BUF_TYPE_META;
		p[2] = 4;
		p[3] = LO32(isp_meta_gpu[i]);
		p[4] = HI32(isp_meta_gpu[i]);
		p[5] = ISP_META_SIZE;
		if ((r = isp_cmd_wait(ISP_CMD_SEND_BUFFER, p, 6)) != 0)
			return (r);
	}
	for (i = 0; i < ISP_NFRAME; i++) {
		if (isp_frame_bo[i] == NULL &&
		    (r = amdgpu_isp_fbsd_alloc(ISP_FRAME_ALLOC, &isp_frame_bo[i],
		    &isp_frame_gpu[i], &isp_frame_cpu[i])) != 0)
			return (r);
		memset(isp_frame_cpu[i], 0, ISP_FRAME_ALLOC);
		uv = isp_frame_gpu[i] + ISP_Y_SIZE;
		memset(p, 0, sizeof(p));
		p[0] = ISP_BUF_TYPE_FRAME;
		p[2] = 4;
		p[3] = LO32(isp_frame_gpu[i]);
		p[4] = HI32(isp_frame_gpu[i]);
		if (isp_yuyv) {
			p[5] = ISP_YUYV_SIZE;
		} else {
			p[5] = ISP_Y_SIZE;
			p[6] = LO32(uv);
			p[7] = HI32(uv);
			p[8] = ISP_Y_SIZE / 2;
		}
		if ((r = isp_cmd_wait(ISP_CMD_SEND_BUFFER, p, 9)) != 0)
			return (r);
	}
	return (isp_cmd_wait(ISP_CMD_START_STREAM, NULL, 0));
}

/* Disable the preview channel and stop the stream. */
static void
isp_stream_stop_cmds(void)
{
	uint32_t dis[2] = { 0, 0 };

	isp_cmd_wait(ISP_CMD_ENABLE_OUT_CHAN, dis, nitems(dis));
	isp_cmd_wait(ISP_CMD_STOP_STREAM, NULL, 0);
}

/* Test: set up a stream, take 90 frames, stop. */
static int
isp_one_frame(void)
{
	int i, r;

	if ((r = isp_stream_setup()) != 0)
		return (r);
	/* Stream 90 frames (3 s), giving each buffer back as it completes. */
	isp_nframes = 0;
	isp_requeue = 1;
	for (i = 0; i < 200 && isp_nframes < 90; i++)
		if ((r = isp_poll_resp_type(2000, ISP_RESP_FRAME_DONE)) != 0)
			break;
	isp_requeue = 0;
	printf("amdisp: %d frames\n", isp_nframes);
	isp_stream_stop_cmds();
	return (r);
}

/* First command: give the firmware its working memory, as Linux does. */
static int
isp_send_pool(void)
{
	uint32_t p[6];
	int r;

	if (isp_pool_bo == NULL) {
		r = amdgpu_isp_fbsd_alloc(ISP_POOL_SIZE, &isp_pool_bo, &isp_pool_gpu,
		    &isp_pool_cpu);
		if (r != 0) {
			printf("amdisp: pool allocation failed: %d\n", r);
			return (r);
		}
	}
	p[0] = ISP_BUF_TYPE_POOL;
	p[1] = 0;
	p[2] = 4;
	p[3] = LO32(isp_pool_gpu);
	p[4] = HI32(isp_pool_gpu);
	p[5] = ISP_POOL_SIZE;
	r = isp_send_cmd(ISP_CMD_SEND_BUFFER, p, nitems(p));
	if (r != 0)
		return (r);
	return (isp_poll_resp(500));
}

static int
isp_boot(void)
{
	uint32_t prev, v;
	int i, r;

	r = amdgpu_isp_fbsd_alloc(16 * 1024 * 1024, &isp_ring_bo, &isp_ring_gpu, &isp_ring_cpu);
	if (r == 0)
		r = amdgpu_isp_fbsd_alloc(ISP_LOG_SIZE, &isp_log_bo, &isp_log_gpu,
		    &isp_log_cpu);
	if (r != 0) {
		printf("amdisp: buffer allocation failed: %d\n", r);
		isp_free();
		return (r);
	}
	memset(isp_ring_cpu, 0, 16 * 1024 * 1024);
	memset(isp_log_cpu, 0, ISP_LOG_SIZE);
	printf("amdisp: rings at GPU 0x%llx, log at GPU 0x%llx\n",
	    (unsigned long long)isp_ring_gpu, (unsigned long long)isp_log_gpu);

	isp_smu(AMDGPU_ISP_FBSD_POWER_UP, 0x7ff);
	isp_smu(AMDGPU_ISP_FBSD_XCLK, 0x314);
	isp_smu(AMDGPU_ISP_FBSD_ICLK, 0x314);

	ISP_WR(ISP_TILES, 0x7ff);
	ISP_WR(ISP_CCPU_CTRL, ISP_RD(ISP_CCPU_CTRL) | 0x40000);
	ISP_WR(ISP_CCPU_RUN, ISP_RD(ISP_CCPU_RUN) | 0x1);
	for (i = 0; i < nitems(isp_rings); i++)
		isp_ring_set(isp_rings[i].reg,
		    isp_ring_gpu + isp_rings[i].slot * ISP_RING_SLOT,
		    ISP_RING_SIZE);
	isp_ring_set(ISP_LOG_RING, isp_log_gpu, ISP_LOG_SIZE);

	ISP_WR(ISP_BOOT_STATUS, 0);
	ISP_WR(ISP_CCPU_RUN, ISP_RD(ISP_CCPU_RUN) & ~0x1);
	ISP_WR(ISP_CCPU_CTRL, ISP_RD(ISP_CCPU_CTRL) & ~0x40000);

	prev = 0xffffffff;
	for (i = 0; i < 1000; i++) {
		v = ISP_RD(ISP_BOOT_STATUS);
		if (v != prev)
			printf("amdisp: boot status 0x%x after %d ms\n", v, i);
		prev = v;
		if (v == ISP_BOOT_READY)
			break;
		MDELAY(1);
	}
	if (v != ISP_BOOT_READY) {
		printf("amdisp: firmware did not boot (status 0x%x)\n", v);
		return (ETIMEDOUT);
	}
	ISP_WR(ISP_INT_ENABLE, 0x410000);
	isp_cmd_seq = 0;
	printf("amdisp: firmware booted\n");
	return (0);
}

static void
isp_stop(void)
{
	ISP_WR(ISP_INT_ENABLE, 0);
	ISP_WR(ISP_CCPU_CTRL, ISP_RD(ISP_CCPU_CTRL) | 0x40000);
	ISP_WR(ISP_CCPU_RUN, ISP_RD(ISP_CCPU_RUN) | 0x1);
	ISP_WR(ISP_CCPU_RUN, 0);
	ISP_WR(ISP_TILES, 0);
	isp_smu(AMDGPU_ISP_FBSD_POWER_DOWN, 0x7ff);
	isp_free();
	printf("amdisp: stopped\n");
}

static int
isp_boot_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	val = isp_booted;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (amdgpu_isp_fbsd_present() != 0)
		return (ENXIO);
	if (val != 0 && !isp_booted) {
		/* A failed boot still holds power and buffers: =0 releases. */
		isp_booted = 1;
		error = isp_boot();
	} else if (val == 0 && isp_booted) {
		isp_stop();
		isp_booted = 0;
	}
	return (error);
}
static int
isp_cmd_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	val = 0;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (amdgpu_isp_fbsd_present() != 0 || !isp_booted)
		return (ENXIO);
	if (val == 1)
		return (isp_send_pool());
	if (val == 2)
		return (isp_one_frame());
	return (EINVAL);
}
SYSCTL_PROC(_hw_amdisp, OID_AUTO, cmd,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, isp_cmd_sysctl, "I",
    "ISP command test: 1 sends the memory pool, 2 sets up a stream and waits for one frame");

/* The last finished frame, NV12 1920x1080, for a check from user space. */
static int
isp_frame_sysctl(SYSCTL_HANDLER_ARGS)
{
	if (isp_last_frame < 0 || isp_frame_cpu[isp_last_frame] == NULL)
		return (ENOENT);
	return (SYSCTL_OUT(req, isp_frame_cpu[isp_last_frame],
	    ISP_CUR_SIZE));
}
/* The firmware log ring, raw, for strings(1). */
static int
isp_log_sysctl(SYSCTL_HANDLER_ARGS)
{
		if (isp_log_cpu == NULL)
		return (ENOENT);
	if (isp_booted)
		printf("amdisp: log ring rptr 0x%x wptr 0x%x\n",
		    ISP_RD(ISP_LOG_RING + 0xc),
		    ISP_RD(ISP_LOG_RING + 0x10));
	return (SYSCTL_OUT(req, isp_log_cpu, ISP_LOG_SIZE));
}
SYSCTL_PROC(_hw_amdisp, OID_AUTO, log,
    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0, isp_log_sysctl,
    "", "ISP firmware log ring");

SYSCTL_PROC(_hw_amdisp, OID_AUTO, frame,
    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0, isp_frame_sysctl,
    "", "Last ISP frame (NV12 1920x1080)");

SYSCTL_PROC(_hw_amdisp, OID_AUTO, boot,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, isp_boot_sysctl, "I",
    "ISP boot test: 1 boots the ISP firmware, 0 stops it");


/*
 * video(4) driver.  One mode: NV12 1920x1080 at 30 frames per second.  A
 * kernel thread reads the response ring every millisecond while streaming
 * (the ISP interrupt is not routed through amdgpu yet).
 */
#define	ISP_FOURCC_NV12		0x3231564e	/* 'NV12' */
#define	ISP_FOURCC_YUYV		0x56595559	/* 'YUYV' */

static struct proc *isp_thread;
static volatile int isp_streaming;

static void
isp_poll_thread(void *arg __unused)
{
	while (isp_streaming) {
		isp_service();
		pause("amdisp", 1);
	}
	isp_thread = NULL;
	wakeup(&isp_thread);
	kproc_exit(0);
}

static void
amdisp_fill_format(struct video_format *fmt, int yuyv)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->pixelformat = yuyv ? ISP_FOURCC_YUYV : ISP_FOURCC_NV12;
	fmt->width = ISP_WIDTH;
	fmt->height = ISP_HEIGHT;
	fmt->bytesperline = yuyv ? ISP_WIDTH * 2 : ISP_WIDTH;
	fmt->sizeimage = yuyv ? ISP_YUYV_SIZE : ISP_FRAME_SIZE;
	fmt->field = 1;		/* V4L2_FIELD_NONE */
	fmt->colorspace = 8;	/* V4L2_COLORSPACE_SRGB */
	strlcpy(fmt->description, yuyv ? "YUYV 4:2:2" : "Y/UV 4:2:0",
	    sizeof(fmt->description));
}

static int
amdisp_querycap(device_t dev, struct video_caps *caps)
{
	memset(caps, 0, sizeof(*caps));
	strlcpy(caps->driver, "amdisp", sizeof(caps->driver));
	strlcpy(caps->card, "AMD ISP 4.1 camera", sizeof(caps->card));
	strlcpy(caps->bus_info, "platform:amdisp", sizeof(caps->bus_info));
	caps->version = (1 << 16);
	caps->capabilities = VIDEO_CAP_CAPTURE | VIDEO_CAP_READWRITE |
	    VIDEO_CAP_STREAMING;
	return (0);
}

static int
amdisp_enum_format(device_t dev, uint32_t index, struct video_format *fmt)
{
	if (index > 1)
		return (EINVAL);
	amdisp_fill_format(fmt, index == 1);
	return (0);
}

static int
amdisp_get_format(device_t dev, struct video_format *fmt)
{
	amdisp_fill_format(fmt, isp_yuyv);
	return (0);
}

/* Any request gets the closest supported mode: YUYV if asked, else NV12. */
static int
amdisp_try_format(device_t dev, struct video_format *fmt)
{
	amdisp_fill_format(fmt, fmt->pixelformat == ISP_FOURCC_YUYV);
	return (0);
}

static int
amdisp_set_format(device_t dev, const struct video_format *fmt)
{
	if (isp_streaming)
		return (EBUSY);
	isp_yuyv = (fmt->pixelformat == ISP_FOURCC_YUYV);
	return (0);
}

static int
amdisp_enum_framesizes(device_t dev, struct video_frmsizeenum *fs)
{
	if (fs->index != 0 || (fs->pixelformat != ISP_FOURCC_NV12 &&
	    fs->pixelformat != ISP_FOURCC_YUYV))
		return (EINVAL);
	fs->type = 1;	/* V4L2_FRMSIZE_TYPE_DISCRETE */
	fs->discrete.width = ISP_WIDTH;
	fs->discrete.height = ISP_HEIGHT;
	return (0);
}

static int
amdisp_enum_frameintervals(device_t dev, struct video_frmivalenum *fi)
{
	if (fi->index != 0 || (fi->pixelformat != ISP_FOURCC_NV12 &&
	    fi->pixelformat != ISP_FOURCC_YUYV) ||
	    fi->width != ISP_WIDTH || fi->height != ISP_HEIGHT)
		return (EINVAL);
	fi->type = 1;	/* V4L2_FRMIVAL_TYPE_DISCRETE */
	fi->discrete.numerator = 1;
	fi->discrete.denominator = 30;
	return (0);
}

static int
amdisp_get_parm(device_t dev, struct video_fract *fract)
{
	fract->numerator = 1;
	fract->denominator = 30;
	return (0);
}

static int
amdisp_enum_input(device_t dev, uint32_t index, struct video_input *inp)
{
	if (index != 0)
		return (EINVAL);
	memset(inp, 0, sizeof(*inp));
	strlcpy(inp->name, "Camera", sizeof(inp->name));
	inp->type = VIDEO_INPUT_TYPE_CAMERA;
	return (0);
}

static int
amdisp_get_input(device_t dev, uint32_t *index)
{
	*index = 0;
	return (0);
}

static int
amdisp_set_input(device_t dev, uint32_t index)
{
	return (index == 0 ? 0 : EINVAL);
}

/*
 * AMD GPIO 85 enables the camera module (Linux amdgpu's "enable_isp"
 * lookup).  Linux raises it at each stream start and drops it at each stop;
 * left high between streams, the next SET_STREAM_CONFIG gets no answer.
 */
#define	ISP_ENABLE_GPIO		85

static void
isp_enable_gpio(int on)
{
	device_t gpio;

	gpio = devclass_get_device(devclass_find("gpio"), 0);
	if (gpio == NULL) {
		printf("amdisp: no gpio0 (kldload amdgpio)\n");
		return;
	}
	GPIO_PIN_SETFLAGS(gpio, ISP_ENABLE_GPIO, GPIO_PIN_OUTPUT);
	GPIO_PIN_SET(gpio, ISP_ENABLE_GPIO, on ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

static int
amdisp_start_stream(device_t dev)
{
	int r;

	if (isp_booted)
		return (EBUSY);	/* the sysctl test owns the ISP */
	isp_enable_gpio(0);
	MDELAY(10);
	isp_enable_gpio(1);
	MDELAY(10);
	isp_booted = 1;
	if ((r = isp_boot()) != 0 || (r = isp_stream_setup()) != 0) {
		isp_stop();
		isp_booted = 0;
		isp_enable_gpio(0);
		return (r);
	}
	isp_video_seq = 0;
	isp_requeue = 1;
	isp_streaming = 1;
	r = kproc_create(isp_poll_thread, NULL, &isp_thread, 0, 0, "amdisp");
	if (r != 0) {
		isp_streaming = 0;
		isp_requeue = 0;
		isp_stream_stop_cmds();
		isp_stop();
		isp_booted = 0;
	}
	return (r);
}

static void
amdisp_stop_stream(device_t dev)
{
	if (!isp_streaming)
		return;
	isp_streaming = 0;
	while (isp_thread != NULL)
		tsleep(&isp_thread, 0, "amdispst", hz / 10);
	isp_requeue = 0;
	isp_stream_stop_cmds();
	isp_stop();
	isp_booted = 0;
	isp_enable_gpio(0);
}

static void
amdisp_identify(driver_t *driver, device_t parent)
{
	if (device_find_child(parent, "amdisp", DEVICE_UNIT_ANY) == NULL)
		BUS_ADD_CHILD(parent, 0, "amdisp", DEVICE_UNIT_ANY);
}

static int
amdisp_probe(device_t dev)
{
	if (amdgpu_isp_fbsd_present() != 0)
		return (ENXIO);
	device_set_desc(dev, "AMD ISP 4.1 camera (test)");
	return (BUS_PROBE_NOWILDCARD);
}

static int
amdisp_attach(device_t dev)
{
	return (video_register(dev, &isp_vd));
}

static int
amdisp_detach(device_t dev)
{
	amdisp_stop_stream(dev);
	if (isp_booted) {
		isp_stop();
		isp_booted = 0;
	}
	if (isp_vd != NULL) {
		video_unregister(isp_vd);
		isp_vd = NULL;
	}
	return (0);
}

static device_method_t amdisp_methods[] = {
	DEVMETHOD(device_identify,	amdisp_identify),
	DEVMETHOD(device_probe,		amdisp_probe),
	DEVMETHOD(device_attach,	amdisp_attach),
	DEVMETHOD(device_detach,	amdisp_detach),

	DEVMETHOD(video_querycap,	amdisp_querycap),
	DEVMETHOD(video_enum_format,	amdisp_enum_format),
	DEVMETHOD(video_get_format,	amdisp_get_format),
	DEVMETHOD(video_try_format,	amdisp_try_format),
	DEVMETHOD(video_set_format,	amdisp_set_format),
	DEVMETHOD(video_enum_framesizes, amdisp_enum_framesizes),
	DEVMETHOD(video_enum_frameintervals, amdisp_enum_frameintervals),
	DEVMETHOD(video_get_parm,	amdisp_get_parm),
	DEVMETHOD(video_set_parm,	amdisp_get_parm),
	DEVMETHOD(video_enum_input,	amdisp_enum_input),
	DEVMETHOD(video_get_input,	amdisp_get_input),
	DEVMETHOD(video_set_input,	amdisp_set_input),
	DEVMETHOD(video_start_stream,	amdisp_start_stream),
	DEVMETHOD(video_stop_stream,	amdisp_stop_stream),

	DEVMETHOD_END
};

static driver_t amdisp_driver = {
	"amdisp",
	amdisp_methods,
	0,
};

DRIVER_MODULE(amdisp, nexus, amdisp_driver, NULL, NULL);
MODULE_VERSION(amdisp, 1);
MODULE_DEPEND(amdisp, video, 1, 1, 1);
MODULE_DEPEND(amdisp, gpiobus, 1, 1, 1);
MODULE_DEPEND(amdisp, amdgpu_isp, AMDGPU_ISP_FBSD_VERSION,
    AMDGPU_ISP_FBSD_VERSION, AMDGPU_ISP_FBSD_VERSION);
