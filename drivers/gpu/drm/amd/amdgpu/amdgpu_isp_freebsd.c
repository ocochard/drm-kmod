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
#include "ivsrcid/isp/irqsrcs_isp_4_1.h"
#include "soc15_ih_clientid.h"
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

int
amdgpu_isp_fbsd_metrics(void *buf, size_t len, uint64_t *features)
{
	void *table = NULL;
	int size;

	if (isp_adev == NULL)
		return (-ENXIO);
	if (smu_cmn_get_enabled_mask(isp_adev->powerplay.pp_handle,
	    features) != 0)
		*features = 0;
	size = amdgpu_dpm_get_gpu_metrics(isp_adev, &table);
	if (size <= 0 || table == NULL)
		return (size < 0 ? size : -ENODATA);
	if ((size_t)size > len)
		size = len;
	memcpy(buf, table, size);
	return (size);
}

static void (*isp_intr_fn)(void *);
static void *isp_intr_arg;
static bool isp_intr_registered;

void
amdgpu_isp_fbsd_set_intr(void (*fn)(void *), void *arg)
{
	isp_intr_arg = arg;
	isp_intr_fn = fn;
}

static int
isp_irq_set(struct amdgpu_device *adev, struct amdgpu_irq_src *src,
    unsigned type, enum amdgpu_interrupt_state state)
{
	return (0);	/* the camera driver enables its interrupts itself */
}

static int
isp_irq_process(struct amdgpu_device *adev, struct amdgpu_irq_src *src,
    struct amdgpu_iv_entry *entry)
{
	void (*fn)(void *) = isp_intr_fn;

	if (fn != NULL)
		fn(isp_intr_arg);
	return (1);
}

static const struct amdgpu_irq_src_funcs isp_irq_funcs = {
	.set = isp_irq_set,
	.process = isp_irq_process,
};

static struct amdgpu_irq_src isp_irq_src = {
	.funcs = &isp_irq_funcs,
};

void
amdgpu_isp_freebsd_attach(struct amdgpu_device *adev)
{
	static const unsigned int srcid[] = {
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT9,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT10,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT11,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT12,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT13,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT14,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT15,
		ISP_4_1__SRCID__ISP_RINGBUFFER_WPT16,
	};
	int i, r;

	isp_adev = adev;
	/* hw_init also runs on resume: register the sources once. */
	if (isp_intr_registered)
		return;
	for (i = 0; i < nitems(srcid); i++) {
		r = amdgpu_irq_add_id(adev, SOC15_IH_CLIENTID_ISP, srcid[i],
		    &isp_irq_src);
		if (r != 0)
			DRM_ERROR("ISP: cannot register interrupt source %u: %d\n",
			    srcid[i], r);
	}
	isp_intr_registered = true;
}

void
amdgpu_isp_freebsd_detach(struct amdgpu_device *adev)
{
	isp_adev = NULL;
}
