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
static struct amdgpu_bo *isp_ring_bo, *isp_log_bo;
static uint64_t isp_ring_gpu, isp_log_gpu;
static void *isp_ring_cpu, *isp_log_cpu;
static int isp_booted;

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
