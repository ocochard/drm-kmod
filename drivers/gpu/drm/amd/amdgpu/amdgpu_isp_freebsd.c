/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Olivier Cochard <olivier@FreeBSD.org>
 */

/*
 * Glue for the FreeBSD AMD ISP 4.1 camera driver (amdisp.ko): register
 * access, GPU buffers and SMU messages.  In Linux the camera driver gets
 * these through child devices that FreeBSD's LinuxKPI does not create.
 */

#include "amdgpu.h"
#include "amdgpu_isp.h"
/* smu_cmn.h exposes its helpers only to the SMU layers. */
#define	SWSMU_CODE_LAYER_L2
#include "amdgpu_smu.h"
#include "smu_cmn.h"

#include <sys/types.h>
#include <sys/module.h>

#include "amdgpu_isp_freebsd.h"

MODULE_VERSION(amdgpu_isp, AMDGPU_ISP_FBSD_VERSION);

static struct amdgpu_device *isp_adev;

int
amdgpu_isp_fbsd_present(void)
{
	return (isp_adev != NULL ? 0 : ENXIO);
}

uint32_t
amdgpu_isp_fbsd_rd(uint32_t off)
{
	struct amdgpu_device *adev = isp_adev;

	return (RREG32(off >> 2));
}

void
amdgpu_isp_fbsd_wr(uint32_t off, uint32_t val)
{
	struct amdgpu_device *adev = isp_adev;

	WREG32(off >> 2, val);
}

int
amdgpu_isp_fbsd_alloc(size_t size, void **handle, uint64_t *gpu, void **cpu)
{
	int r;

	if (isp_adev == NULL)
		return (ENXIO);
	r = amdgpu_bo_create_kernel(isp_adev, size, PAGE_SIZE,
	    AMDGPU_GEM_DOMAIN_GTT, (struct amdgpu_bo **)handle, gpu, cpu);
	return (-r);
}

void
amdgpu_isp_fbsd_free(void **handle, uint64_t *gpu, void **cpu)
{
	amdgpu_bo_free_kernel((struct amdgpu_bo **)handle, gpu, cpu);
}

int
amdgpu_isp_fbsd_smu(int req, uint32_t arg)
{
	static const enum smu_message_type msg[] = {
		[AMDGPU_ISP_FBSD_POWER_UP] = SMU_MSG_PowerUpIspByTile,
		[AMDGPU_ISP_FBSD_POWER_DOWN] = SMU_MSG_PowerDownIspByTile,
		[AMDGPU_ISP_FBSD_XCLK] = SMU_MSG_SetHardMinIspxclkByFreq,
		[AMDGPU_ISP_FBSD_ICLK] = SMU_MSG_SetHardMinIspiclkByFreq,
	};

	if (isp_adev == NULL)
		return (ENXIO);
	if (req < 0 || req >= nitems(msg))
		return (EINVAL);
	return (-smu_cmn_send_smc_msg_with_param(isp_adev->powerplay.pp_handle,
	    msg[req], arg, NULL));
}

void
amdgpu_isp_freebsd_attach(struct amdgpu_device *adev)
{
	isp_adev = adev;
}

void
amdgpu_isp_freebsd_detach(struct amdgpu_device *adev)
{
	isp_adev = NULL;
}
