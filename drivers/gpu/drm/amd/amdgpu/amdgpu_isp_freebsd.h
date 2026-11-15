/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Olivier Cochard <olivier@FreeBSD.org>
 */

/*
 * Services amdgpu gives the FreeBSD AMD ISP 4.1 camera driver (amdisp.ko).
 * Plain C types only, so amdisp builds without the LinuxKPI headers.
 */

#ifndef _AMDGPU_ISP_FREEBSD_H_
#define	_AMDGPU_ISP_FREEBSD_H_

#include <sys/types.h>

#define	AMDGPU_ISP_FBSD_VERSION		1

/* SMU requests. */
#define	AMDGPU_ISP_FBSD_POWER_UP	0	/* PowerUpIspByTile */
#define	AMDGPU_ISP_FBSD_POWER_DOWN	1	/* PowerDownIspByTile */
#define	AMDGPU_ISP_FBSD_XCLK		2	/* SetHardMinIspxclkByFreq */
#define	AMDGPU_ISP_FBSD_ICLK		3	/* SetHardMinIspiclkByFreq */

/* 0 when amdgpu has an ISP 4.1 block ready, ENXIO otherwise. */
int		amdgpu_isp_fbsd_present(void);
uint32_t	amdgpu_isp_fbsd_rd(uint32_t off);
void		amdgpu_isp_fbsd_wr(uint32_t off, uint32_t val);
/* GPU buffer in system memory (GTT), mapped for the CPU. */
int		amdgpu_isp_fbsd_alloc(size_t size, void **handle,
		    uint64_t *gpu, void **cpu);
void		amdgpu_isp_fbsd_free(void **handle, uint64_t *gpu, void **cpu);
int		amdgpu_isp_fbsd_smu(int req, uint32_t arg);

#endif /* _AMDGPU_ISP_FREEBSD_H_ */
