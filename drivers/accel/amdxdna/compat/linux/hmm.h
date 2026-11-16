/* SPDX-License-Identifier: GPL-2.0 */
/*
 * FreeBSD: LinuxKPI has no HMM and no MMU interval notifiers.  amdxdna
 * uses them only to track user mappings for IOMMU SVA, which FreeBSD
 * does not support either.  Keep the types so the driver builds; the
 * operations fail or do nothing.
 */
#ifndef _AMDXDNA_COMPAT_HMM_H_
#define	_AMDXDNA_COMPAT_HMM_H_

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mmu_notifier.h>

struct mm_struct;
struct mmu_interval_notifier;

enum mmu_notifier_event {
	MMU_NOTIFY_UNMAP = 0,
};

struct mmu_notifier_range {
	unsigned long		start;
	unsigned long		end;
	enum mmu_notifier_event	event;
};

struct mmu_interval_notifier_ops {
	bool (*invalidate)(struct mmu_interval_notifier *,
	    const struct mmu_notifier_range *, unsigned long);
};

struct mmu_interval_notifier {
	struct mm_struct	*mm;
	unsigned long		invalidate_seq;
};

#define	HMM_PFN_REQ_FAULT		(1UL << 63)
#define	HMM_RANGE_DEFAULT_TIMEOUT	1000

struct hmm_range {
	struct mmu_interval_notifier	*notifier;
	unsigned long			notifier_seq;
	unsigned long			start;
	unsigned long			end;
	unsigned long			*hmm_pfns;
	unsigned long			default_flags;
	unsigned long			pfn_flags_mask;
	void				*dev_private_owner;
};

static inline bool
mmu_notifier_range_blockable(const struct mmu_notifier_range *range)
{
	return (true);
}

static inline void
mmu_interval_set_seq(struct mmu_interval_notifier *mni, unsigned long seq)
{
	mni->invalidate_seq = seq;
}

static inline int
mmu_interval_notifier_insert_locked(struct mmu_interval_notifier *mni,
    struct mm_struct *mm, unsigned long start, unsigned long length,
    const struct mmu_interval_notifier_ops *ops)
{
	return (-EOPNOTSUPP);
}

static inline void
mmu_interval_notifier_remove(struct mmu_interval_notifier *mni)
{
}

static inline unsigned long
mmu_interval_read_begin(struct mmu_interval_notifier *mni)
{
	return (mni->invalidate_seq);
}

static inline bool
mmu_interval_read_retry(struct mmu_interval_notifier *mni, unsigned long seq)
{
	return (false);
}

static inline int
hmm_range_fault(struct hmm_range *range)
{
	return (-EOPNOTSUPP);
}

#endif
