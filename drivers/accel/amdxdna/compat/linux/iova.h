/* SPDX-License-Identifier: GPL-2.0 */
/*
 * FreeBSD: no IOVA allocator in LinuxKPI.  amdxdna only embeds the
 * domain in struct amdxdna_dev; force_iova mode is not supported.
 */
#ifndef _AMDXDNA_COMPAT_IOVA_H_
#define	_AMDXDNA_COMPAT_IOVA_H_

struct iova_domain {
	unsigned long	granule;
};

#endif
