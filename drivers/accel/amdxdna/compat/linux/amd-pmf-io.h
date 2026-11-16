/* SPDX-License-Identifier: GPL-2.0 */
/*
 * FreeBSD: no amd-pmf driver.  Only the NPU metrics layout is needed;
 * CONFIG_AMD_PMF is unset so amd_pmf_get_npu_data() is never called.
 */
#ifndef _AMDXDNA_COMPAT_AMD_PMF_IO_H_
#define	_AMDXDNA_COMPAT_AMD_PMF_IO_H_

#include <linux/types.h>

struct amd_pmf_npu_metrics {
	u16 npuclk_freq;
	u16 npu_busy[8];
	u16 npu_power;
	u16 mpnpuclk_freq;
	u16 npu_reads;
	u16 npu_writes;
};

#endif
