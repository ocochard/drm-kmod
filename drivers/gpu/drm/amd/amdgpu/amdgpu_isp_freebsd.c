/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Olivier Cochard <olivier@FreeBSD.org>
 */

/*
 * AMD ISP 4.1 boot test (first step of a FreeBSD camera driver).
 *
 * sysctl hw.amdgpu.isp_boot=1 powers the ISP up, programs its rings and
 * releases its processor, then reports the boot status; =0 stops it.  The
 * register sequence was measured on Linux (mmiotrace), not taken from its
 * code.  The ISP enable line (AMD GPIO 85) must be driven high first, for
 * example with gpioctl(8).
 */

#include <linux/delay.h>

#include "amdgpu.h"
#include "amdgpu_drv.h"
#include "amdgpu_isp.h"
/* smu_cmn.h exposes its helpers only to the SMU layers. */
#define	SWSMU_CODE_LAYER_L2
#include "amdgpu_smu.h"
#include "smu_cmn.h"

#include <sys/types.h>
#include <sys/sysctl.h>

#define	ISP_RD(adev, off)	RREG32((off) >> 2)
#define	ISP_WR(adev, off, v)	WREG32((off) >> 2, (v))

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

static struct amdgpu_device *isp_adev;
static struct amdgpu_bo *isp_ring_bo, *isp_log_bo, *isp_pool_bo;
static uint64_t isp_ring_gpu, isp_log_gpu, isp_pool_gpu;
static void *isp_ring_cpu, *isp_log_cpu, *isp_pool_cpu;
static int isp_booted;
static uint32_t isp_cmd_seq;
static struct amdgpu_bo *isp_meta_bo[ISP_NMETA], *isp_frame_bo[ISP_NFRAME];
static uint64_t isp_meta_gpu[ISP_NMETA], isp_frame_gpu[ISP_NFRAME];
static void *isp_meta_cpu[ISP_NMETA], *isp_frame_cpu[ISP_NFRAME];
static int isp_last_frame = -1;

static void
isp_ring_set(struct amdgpu_device *adev, uint32_t reg, uint64_t gpu,
    uint32_t size)
{
	ISP_WR(adev, reg + 0xc, 0);		/* read pointer */
	ISP_WR(adev, reg + 0x10, 0);		/* write pointer */
	ISP_WR(adev, reg, lower_32_bits(gpu));
	ISP_WR(adev, reg + 0x4, upper_32_bits(gpu));
	ISP_WR(adev, reg + 0x8, size);
}

static void
isp_smu(struct amdgpu_device *adev, enum smu_message_type msg, uint32_t arg)
{
	int r;

	r = smu_cmn_send_smc_msg_with_param(adev->powerplay.pp_handle, msg,
	    arg, NULL);
	if (r != 0)
		printf("amdisp: SMU message %d failed: %d\n", msg, r);
}

static void
isp_free(struct amdgpu_device *adev)
{
	amdgpu_bo_free_kernel(&isp_ring_bo, &isp_ring_gpu, &isp_ring_cpu);
	amdgpu_bo_free_kernel(&isp_log_bo, &isp_log_gpu, &isp_log_cpu);
	amdgpu_bo_free_kernel(&isp_pool_bo, &isp_pool_gpu, &isp_pool_cpu);
	for (int i = 0; i < ISP_NMETA; i++)
		amdgpu_bo_free_kernel(&isp_meta_bo[i], &isp_meta_gpu[i],
		    &isp_meta_cpu[i]);
	for (int i = 0; i < ISP_NFRAME; i++)
		amdgpu_bo_free_kernel(&isp_frame_bo[i], &isp_frame_gpu[i],
		    &isp_frame_cpu[i]);
	isp_last_frame = -1;
}

/*
 * A command or response entry is 16 32-bit words: sequence number,
 * identifier, 12 payload words, a zero word and a checksum, the 32-bit sum
 * of the first 15 words.
 */
static int
isp_send_cmd(struct amdgpu_device *adev, uint32_t id, const uint32_t *payload,
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

	rptr = ISP_RD(adev, ISP_CMD_RPTR);
	wptr = ISP_RD(adev, ISP_CMD_WPTR);
	if ((wptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE == rptr)
		return (ENOSPC);
	ring = (uint8_t *)isp_ring_cpu + ISP_CMD_SLOT * ISP_RING_SLOT;
	memcpy(ring + wptr, e, sizeof(e));
	wmb();
	ISP_WR(adev, ISP_CMD_WPTR, (wptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE);
	printf("amdisp: sent cmd 0x%08x seq %u at ring offset 0x%x\n", id,
	    e[0], wptr);
	return (0);
}

static void isp_frame_done(uint64_t meta_gpu);
static int isp_send_pool(struct amdgpu_device *adev);

/* Poll the response ring for up to timeout_ms, printing each response. */
static int
isp_poll_resp_type(struct amdgpu_device *adev, int timeout_ms, uint32_t want)
{
	uint32_t e[ISP_ENTRY_SIZE / 4], rptr, wptr, st, sum;
	uint8_t *ring;
	int i, n, t;

	ring = (uint8_t *)isp_ring_cpu + ISP_RESP_SLOT * ISP_RING_SLOT;
	for (n = 0, t = 0; t < timeout_ms; t++) {
		st = ISP_RD(adev, ISP_INT_STATUS);
		if (st != 0) {
			ISP_WR(adev, ISP_INT_ACK, st);
			printf("amdisp: interrupt status 0x%x at %d ms\n", st,
			    t);
		}
		rptr = ISP_RD(adev, ISP_RESP_RPTR);
		wptr = ISP_RD(adev, ISP_RESP_WPTR);
		while (rptr != wptr) {
			rmb();
			memcpy(e, ring + rptr, sizeof(e));
			for (sum = 0, i = 0; i < 15; i++)
				sum += e[i];
			printf("amdisp: resp seq %u type 0x%08x w2 0x%x w3 0x%x "
			    "w4 0x%x w5 0x%x checksum %s at %d ms\n", e[0], e[1],
			    e[2], e[3], e[4], e[5], sum == e[15] ? "ok" : "BAD",
			    t);
			rptr = (rptr + ISP_ENTRY_SIZE) % ISP_RING_SIZE;
			ISP_WR(adev, ISP_RESP_RPTR, rptr);
			if (e[1] == ISP_RESP_FRAME_DONE)
				isp_frame_done((uint64_t)e[3] << 32 | e[2]);
			if (want == 0 || e[1] == want)
				n++;
		}
		if (n > 0)
			return (0);
		mdelay(1);
	}
	printf("amdisp: no response after %d ms (cmd rptr 0x%x wptr 0x%x)\n",
	    timeout_ms, ISP_RD(adev, ISP_CMD_RPTR), ISP_RD(adev, ISP_CMD_WPTR));
	return (ETIMEDOUT);
}

static int
isp_poll_resp(struct amdgpu_device *adev, int timeout_ms)
{
	return (isp_poll_resp_type(adev, timeout_ms, 0));
}

/* Send a command and wait for its "command done". */
static int
isp_cmd_wait(struct amdgpu_device *adev, uint32_t id, const uint32_t *p,
    int n)
{
	int r;

	r = isp_send_cmd(adev, id, p, n);
	if (r == 0)
		r = isp_poll_resp_type(adev, 500, ISP_RESP_CMD_DONE);
	return (r);
}

/* A finished frame: the meta info tells which frame buffer it is. */
static void
isp_frame_done(uint64_t meta_gpu)
{
	uint32_t *m, status, w, h;
	uint64_t y;
	uint8_t *f;
	uint64_t sum;
	int i, j;

	for (i = 0; i < ISP_NMETA; i++)
		if (isp_meta_gpu[i] == meta_gpu)
			break;
	if (i == ISP_NMETA) {
		printf("amdisp: frame done with unknown meta 0x%llx\n",
		    (unsigned long long)meta_gpu);
		return;
	}
	m = isp_meta_cpu[i];
	status = m[0x14 / 4];
	w = m[0x34 / 4];
	h = m[0x38 / 4];
	y = (uint64_t)m[0x54 / 4] << 32 | m[0x50 / 4];
	for (j = 0; j < ISP_NFRAME; j++)
		if (isp_frame_gpu[j] == y)
			break;
	printf("amdisp: meta %d: frame %u status %u %ux%u Y 0x%llx -> buffer %d\n",
	    i, m[0], status, w, h, (unsigned long long)y, j);
	if (j == ISP_NFRAME)
		return;
	f = isp_frame_cpu[j];
	for (sum = 0, i = 0; i < ISP_Y_SIZE; i += 4093)
		sum += f[i];
	printf("amdisp: buffer %d mean Y %llu\n", j,
	    (unsigned long long)(sum / (ISP_Y_SIZE / 4093 + 1)));
	isp_last_frame = j;
}

/*
 * Stream set-up as measured on Linux, then wait for one frame: stream
 * configuration, NV12 1920x1080 on the preview channel, eight meta info
 * buffers, four frame buffers, start.
 */
static int
isp_one_frame(struct amdgpu_device *adev)
{
	uint32_t cfg[5] = { 1, 0, 0x5f91, 1, 0 };
	uint32_t prop[7] = { 0, 1, ISP_WIDTH, ISP_HEIGHT, ISP_WIDTH,
	    ISP_WIDTH, 0 };
	uint32_t en[2] = { 0, 1 }, dis[2] = { 0, 0 };
	uint32_t p[9];
	uint64_t uv;
	int i, r;

	if ((r = isp_send_pool(adev)) != 0)
		return (r);
	if ((r = isp_cmd_wait(adev, ISP_CMD_SET_STREAM_CONFIG, cfg,
	    nitems(cfg))) != 0 ||
	    (r = isp_cmd_wait(adev, ISP_CMD_SET_OUT_CHAN_PROP, prop,
	    nitems(prop))) != 0 ||
	    (r = isp_cmd_wait(adev, ISP_CMD_ENABLE_OUT_CHAN, en,
	    nitems(en))) != 0)
		return (r);

	for (i = 0; i < ISP_NMETA; i++) {
		if (isp_meta_bo[i] == NULL &&
		    (r = amdgpu_bo_create_kernel(adev, ISP_META_SIZE,
		    PAGE_SIZE, AMDGPU_GEM_DOMAIN_GTT, &isp_meta_bo[i],
		    &isp_meta_gpu[i], &isp_meta_cpu[i])) != 0)
			return (r);
		memset(isp_meta_cpu[i], 0, ISP_META_SIZE);
		memset(p, 0, sizeof(p));
		p[0] = ISP_BUF_TYPE_META;
		p[2] = 4;
		p[3] = lower_32_bits(isp_meta_gpu[i]);
		p[4] = upper_32_bits(isp_meta_gpu[i]);
		p[5] = ISP_META_SIZE;
		if ((r = isp_cmd_wait(adev, ISP_CMD_SEND_BUFFER, p, 6)) != 0)
			return (r);
	}
	for (i = 0; i < ISP_NFRAME; i++) {
		if (isp_frame_bo[i] == NULL &&
		    (r = amdgpu_bo_create_kernel(adev, ISP_FRAME_SIZE,
		    PAGE_SIZE, AMDGPU_GEM_DOMAIN_GTT, &isp_frame_bo[i],
		    &isp_frame_gpu[i], &isp_frame_cpu[i])) != 0)
			return (r);
		memset(isp_frame_cpu[i], 0, ISP_FRAME_SIZE);
		uv = isp_frame_gpu[i] + ISP_Y_SIZE;
		memset(p, 0, sizeof(p));
		p[0] = ISP_BUF_TYPE_FRAME;
		p[2] = 4;
		p[3] = lower_32_bits(isp_frame_gpu[i]);
		p[4] = upper_32_bits(isp_frame_gpu[i]);
		p[5] = ISP_Y_SIZE;
		p[6] = lower_32_bits(uv);
		p[7] = upper_32_bits(uv);
		p[8] = ISP_Y_SIZE / 2;
		if ((r = isp_cmd_wait(adev, ISP_CMD_SEND_BUFFER, p, 9)) != 0)
			return (r);
	}
	if ((r = isp_cmd_wait(adev, ISP_CMD_START_STREAM, NULL, 0)) != 0)
		return (r);
	r = isp_poll_resp_type(adev, 2000, ISP_RESP_FRAME_DONE);

	isp_cmd_wait(adev, ISP_CMD_ENABLE_OUT_CHAN, dis, nitems(dis));
	isp_cmd_wait(adev, ISP_CMD_STOP_STREAM, NULL, 0);
	return (r);
}

/* First command: give the firmware its working memory, as Linux does. */
static int
isp_send_pool(struct amdgpu_device *adev)
{
	uint32_t p[6];
	int r;

	if (isp_pool_bo == NULL) {
		r = amdgpu_bo_create_kernel(adev, ISP_POOL_SIZE, PAGE_SIZE,
		    AMDGPU_GEM_DOMAIN_GTT, &isp_pool_bo, &isp_pool_gpu,
		    &isp_pool_cpu);
		if (r != 0) {
			printf("amdisp: pool allocation failed: %d\n", r);
			return (r);
		}
	}
	p[0] = ISP_BUF_TYPE_POOL;
	p[1] = 0;
	p[2] = 4;
	p[3] = lower_32_bits(isp_pool_gpu);
	p[4] = upper_32_bits(isp_pool_gpu);
	p[5] = ISP_POOL_SIZE;
	r = isp_send_cmd(adev, ISP_CMD_SEND_BUFFER, p, nitems(p));
	if (r != 0)
		return (r);
	return (isp_poll_resp(adev, 500));
}

static int
isp_boot(struct amdgpu_device *adev)
{
	uint32_t prev, v;
	int i, r;

	r = amdgpu_bo_create_kernel(adev, 16 * 1024 * 1024, PAGE_SIZE,
	    AMDGPU_GEM_DOMAIN_GTT, &isp_ring_bo, &isp_ring_gpu, &isp_ring_cpu);
	if (r == 0)
		r = amdgpu_bo_create_kernel(adev, ISP_LOG_SIZE, PAGE_SIZE,
		    AMDGPU_GEM_DOMAIN_GTT, &isp_log_bo, &isp_log_gpu,
		    &isp_log_cpu);
	if (r != 0) {
		printf("amdisp: buffer allocation failed: %d\n", r);
		isp_free(adev);
		return (r);
	}
	memset(isp_ring_cpu, 0, 16 * 1024 * 1024);
	memset(isp_log_cpu, 0, ISP_LOG_SIZE);
	printf("amdisp: rings at GPU 0x%llx, log at GPU 0x%llx\n",
	    (unsigned long long)isp_ring_gpu, (unsigned long long)isp_log_gpu);

	isp_smu(adev, SMU_MSG_PowerUpIspByTile, 0x7ff);
	isp_smu(adev, SMU_MSG_SetHardMinIspxclkByFreq, 0x314);
	isp_smu(adev, SMU_MSG_SetHardMinIspiclkByFreq, 0x314);

	ISP_WR(adev, ISP_TILES, 0x7ff);
	ISP_WR(adev, ISP_CCPU_CTRL, ISP_RD(adev, ISP_CCPU_CTRL) | 0x40000);
	ISP_WR(adev, ISP_CCPU_RUN, ISP_RD(adev, ISP_CCPU_RUN) | 0x1);
	for (i = 0; i < nitems(isp_rings); i++)
		isp_ring_set(adev, isp_rings[i].reg,
		    isp_ring_gpu + isp_rings[i].slot * ISP_RING_SLOT,
		    ISP_RING_SIZE);
	isp_ring_set(adev, ISP_LOG_RING, isp_log_gpu, ISP_LOG_SIZE);

	ISP_WR(adev, ISP_BOOT_STATUS, 0);
	ISP_WR(adev, ISP_CCPU_RUN, ISP_RD(adev, ISP_CCPU_RUN) & ~0x1);
	ISP_WR(adev, ISP_CCPU_CTRL, ISP_RD(adev, ISP_CCPU_CTRL) & ~0x40000);

	prev = 0xffffffff;
	for (i = 0; i < 1000; i++) {
		v = ISP_RD(adev, ISP_BOOT_STATUS);
		if (v != prev)
			printf("amdisp: boot status 0x%x after %d ms\n", v, i);
		prev = v;
		if (v == ISP_BOOT_READY)
			break;
		mdelay(1);
	}
	if (v != ISP_BOOT_READY) {
		printf("amdisp: firmware did not boot (status 0x%x)\n", v);
		return (ETIMEDOUT);
	}
	ISP_WR(adev, ISP_INT_ENABLE, 0x410000);
	isp_cmd_seq = 0;
	printf("amdisp: firmware booted\n");
	return (0);
}

static void
isp_stop(struct amdgpu_device *adev)
{
	ISP_WR(adev, ISP_INT_ENABLE, 0);
	ISP_WR(adev, ISP_CCPU_CTRL, ISP_RD(adev, ISP_CCPU_CTRL) | 0x40000);
	ISP_WR(adev, ISP_CCPU_RUN, ISP_RD(adev, ISP_CCPU_RUN) | 0x1);
	ISP_WR(adev, ISP_CCPU_RUN, 0);
	ISP_WR(adev, ISP_TILES, 0);
	isp_smu(adev, SMU_MSG_PowerDownIspByTile, 0x7ff);
	isp_free(adev);
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
	if (isp_adev == NULL)
		return (ENXIO);
	if (val != 0 && !isp_booted) {
		/* A failed boot still holds power and buffers: =0 releases. */
		isp_booted = 1;
		error = isp_boot(isp_adev);
	} else if (val == 0 && isp_booted) {
		isp_stop(isp_adev);
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
	if (isp_adev == NULL || !isp_booted)
		return (ENXIO);
	if (val == 1)
		return (isp_send_pool(isp_adev));
	if (val == 2)
		return (isp_one_frame(isp_adev));
	return (EINVAL);
}
SYSCTL_PROC(_hw_amdgpu, OID_AUTO, isp_cmd,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, isp_cmd_sysctl, "I",
    "ISP command test: 1 sends the memory pool, 2 sets up a stream and waits for one frame");

/* The last finished frame, NV12 1920x1080, for a check from user space. */
static int
isp_frame_sysctl(SYSCTL_HANDLER_ARGS)
{
	if (isp_last_frame < 0 || isp_frame_cpu[isp_last_frame] == NULL)
		return (ENOENT);
	return (SYSCTL_OUT(req, isp_frame_cpu[isp_last_frame],
	    ISP_FRAME_SIZE));
}
SYSCTL_PROC(_hw_amdgpu, OID_AUTO, isp_frame,
    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0, isp_frame_sysctl,
    "", "Last ISP frame (NV12 1920x1080)");

SYSCTL_PROC(_hw_amdgpu, OID_AUTO, isp_boot,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, isp_boot_sysctl, "I",
    "ISP boot test: 1 boots the ISP firmware, 0 stops it");

void
amdgpu_isp_freebsd_attach(struct amdgpu_device *adev)
{
	isp_adev = adev;
}

void
amdgpu_isp_freebsd_detach(struct amdgpu_device *adev)
{
	if (isp_booted) {
		isp_stop(adev);
		isp_booted = 0;
	}
	isp_adev = NULL;
}
