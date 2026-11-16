/* SPDX-License-Identifier: GPL-2.0 */
/*
 * FreeBSD: Linux kernel interfaces used by amdxdna that LinuxKPI or the
 * drm-kmod 6.12 core lack.  Force-included before every amdxdna source
 * file.
 */
#ifndef _AMDXDNA_FREEBSD_H_
#define	_AMDXDNA_FREEBSD_H_

#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/conf.h>
#include <vm/vm.h>
#include <vm/vm_object.h>

SYSCTL_DECL(_hw_amdxdna);

#include <linux/types.h>
#include <linux/err.h>
#include <linux/bits.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/xarray.h>
#include <linux/completion.h>
#include <linux/iosys-map.h>
#include <linux/iommu.h>
#include <linux/dma-buf.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <drm/drm_device.h>
#include <drm/drm_gem.h>
#include <drm/drm_managed.h>
#include <drm/gpu_scheduler.h>

/* xarray iterators (Linux 5.x+). */
#ifndef xa_for_each_start
#define	xa_for_each_start(xa, index, entry, start)			\
	for ((entry) = NULL, (index) = (start);				\
	     ((entry) = xa_next(xa, &(index), (entry) != NULL)) != NULL; )
#endif

#ifndef xa_for_each_range
#define	xa_for_each_range(xa, index, entry, first, last)		\
	for ((entry) = NULL, (index) = (first);				\
	     ((entry) = xa_next(xa, &(index), (entry) != NULL)) != NULL && \
	     (index) <= (last); )
#endif

/* Typed allocators (Linux 7.0).  Optional last argument is the gfp. */
#define	__amdxdna_gfp1(gfp, ...)	(gfp)
#define	__amdxdna_gfp(...)						\
	__amdxdna_gfp1(__VA_ARGS__ __VA_OPT__(,) GFP_KERNEL)
#define	__amdxdna_alloc_objs(fn, gfp, type, n)				\
	((type *)fn(size_mul(sizeof(type), (n)), gfp))

#ifndef kzalloc_obj
#define	kzalloc_obj(P, ...)						\
	__amdxdna_alloc_objs(kzalloc, __amdxdna_gfp(__VA_ARGS__), typeof(P), 1)
#endif
#ifndef kzalloc_objs
#define	kzalloc_objs(P, n, ...)						\
	__amdxdna_alloc_objs(kzalloc, __amdxdna_gfp(__VA_ARGS__), typeof(P), n)
#endif
#ifndef kmalloc_obj
#define	kmalloc_obj(P, ...)						\
	__amdxdna_alloc_objs(kmalloc, __amdxdna_gfp(__VA_ARGS__), typeof(P), 1)
#endif
#ifndef kvzalloc_objs
#define	kvzalloc_objs(P, n, ...)						\
	__amdxdna_alloc_objs(kvzalloc, __amdxdna_gfp(__VA_ARGS__), typeof(P), n)
#endif
#ifndef kvmalloc_objs
#define	kvmalloc_objs(P, n, ...)						\
	__amdxdna_alloc_objs(kvmalloc, __amdxdna_gfp(__VA_ARGS__), typeof(P), n)
#endif

#ifndef GENMASK_U64
#define	GENMASK_U64(h, l)	GENMASK_ULL(h, l)
#endif

/* LinuxKPI struct completion has only a done counter. */
#ifndef COMPLETION_INITIALIZER_ONSTACK
#define	COMPLETION_INITIALIZER_ONSTACK(work)	{ .done = 0 }
#endif

/* Only the six standard BARs are visible through LinuxKPI. */
#ifndef PCI_NUM_RESOURCES
#define	PCI_NUM_RESOURCES	(PCIR_MAX_BAR_0 + 1)
#endif

#ifndef IOSYS_MAP_INIT_VADDR
#define	IOSYS_MAP_INIT_VADDR(vaddr_)					\
	{ .vaddr = (vaddr_), .is_iomem = false }
#endif

#define	drmm_kmalloc(dev, size, gfp)	drmm_kzalloc(dev, size, gfp)

/* Linux 6.13 renamed the unlocked vmap helpers. */
#define	drm_gem_vmap(obj, map)		drm_gem_vmap_unlocked(obj, map)
#define	drm_gem_vunmap(obj, map)	drm_gem_vunmap_unlocked(obj, map)

#define	mmput_async(mm)			mmput(mm)

struct page **amdxdna_drm_gem_get_pages(struct drm_gem_object *obj);
void amdxdna_drm_gem_put_pages(struct drm_gem_object *obj,
    struct page **pages, bool dirty, bool accessed);
#define	drm_gem_get_pages(obj)		amdxdna_drm_gem_get_pages(obj)
#define	drm_gem_put_pages(obj, p, d, a)	amdxdna_drm_gem_put_pages(obj, p, d, a)

/* Fault handlers run with the VM object unlocked. */
static inline vm_fault_t
amdxdna_vmf_insert_pfn(struct vm_area_struct *vma, unsigned long addr,
    unsigned long pfn)
{
	vm_fault_t ret;

	VM_OBJECT_WLOCK(vma->vm_obj);
	ret = lkpi_vmf_insert_pfn_prot_locked(vma, addr, pfn,
	    vma->vm_page_prot);
	VM_OBJECT_WUNLOCK(vma->vm_obj);
	return (ret);
}
#define	vmf_insert_pfn(vma, addr, pfn)	amdxdna_vmf_insert_pfn(vma, addr, pfn)

/* vm_insert_pages() at mmap time: FreeBSD maps lazily from ->fault. */
static inline void
vm_flags_mod(struct vm_area_struct *vma, unsigned long set,
    unsigned long clear)
{
	vma->vm_flags = (vma->vm_flags | set) & ~clear;
}

static inline int
vm_insert_pages(struct vm_area_struct *vma, unsigned long addr,
    struct page **pages, unsigned long *num)
{
	*num = 0;
	return (0);
}

/* No prefault: the exporter's ->fault maps each page on first touch. */
static inline vm_fault_t
handle_mm_fault(struct vm_area_struct *vma, unsigned long addr,
    unsigned int flags, void *regs)
{
	return (0);
}

/* drm-kmod has dma-buf mmap only as a file op; call the exporter. */
static inline int
dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma,
    unsigned long pgoff)
{
	if (dmabuf == NULL || vma == NULL || dmabuf->ops->mmap == NULL)
		return (-EINVAL);
	vma->vm_pgoff = pgoff;
	return (dmabuf->ops->mmap(dmabuf, vma));
}

struct amdxdna_dev;
int amdxdna_freebsd_carveout_init(struct amdxdna_dev *xdna);
void amdxdna_freebsd_carveout_fini(struct amdxdna_dev *xdna);

#define	drm_fdinfo_print_size(p, prefix, stat, region, sz)		\
	((void)(p), (void)(prefix), (void)(sz))

#ifndef BIT_U64
#define	BIT_U64(n)	BIT_ULL(n)
#endif

/* LinuxKPI down_interruptible() is not an expression. */
#undef down_interruptible
#define	down_interruptible(_sem)	(sema_wait(&(_sem)->sema), 0)

#ifndef WQ_MEM_RECLAIM
#define	WQ_MEM_RECLAIM	0
#endif

#define	drmm_mutex_init(dev, lock)	({ mutex_init(lock); 0; })

static inline void
amdxdna_drmm_wq_release(struct drm_device *dev, void *wq)
{
	destroy_workqueue(wq);
}

#define	drmm_alloc_ordered_workqueue(dev, fmt, flags, ...)		\
({									\
	struct workqueue_struct *_wq;					\
									\
	_wq = alloc_ordered_workqueue(fmt, flags, ##__VA_ARGS__);	\
	if (_wq == NULL)						\
		_wq = ERR_PTR(-ENOMEM);					\
	else if (drmm_add_action_or_reset(dev, amdxdna_drmm_wq_release,	\
	    _wq) != 0)							\
		_wq = ERR_PTR(-ENOMEM);					\
	_wq;								\
})

#ifndef SYSTEM_SLEEP_PM_OPS
#define	SYSTEM_SLEEP_PM_OPS(s, r)	SET_SYSTEM_SLEEP_PM_OPS(s, r)
#endif
#ifndef RUNTIME_PM_OPS
#define	RUNTIME_PM_OPS(s, r, i)
#endif

static inline void
pm_runtime_set_suspended(struct device *dev)
{
}

/* No IOMMU SVA (PASID) support: every bind fails. */
#define	IOMMU_PASID_INVALID	(-1U)
struct iommu_sva;
struct iommu_group;

static inline struct iommu_sva *
iommu_sva_bind_device(struct device *dev, struct mm_struct *mm)
{
	return (ERR_PTR(-ENODEV));
}

static inline u32
iommu_sva_get_pasid(struct iommu_sva *handle)
{
	return (IOMMU_PASID_INVALID);
}

static inline void
iommu_sva_unbind_device(struct iommu_sva *handle)
{
}

/*
 * drm_sched: amdxdna targets the Linux 7.x API, drm-kmod ships 6.12.
 */
#define	DRM_GPU_SCHED_STAT_NO_HANG	DRM_GPU_SCHED_STAT_NOMINAL
#define	DRM_GPU_SCHED_STAT_RESET	DRM_GPU_SCHED_STAT_NOMINAL

struct drm_sched_init_args {
	const struct drm_sched_backend_ops	*ops;
	struct workqueue_struct			*submit_wq;
	struct workqueue_struct			*timeout_wq;
	u32					num_rqs;
	u32					credit_limit;
	unsigned int				hang_limit;
	long					timeout;
	atomic_t				*score;
	const char				*name;
	struct device				*dev;
};

static inline int
amdxdna_drm_sched_init(struct drm_gpu_scheduler *sched,
    const struct drm_sched_init_args *args)
{
	return (drm_sched_init(sched, args->ops, args->submit_wq,
	    args->num_rqs ? args->num_rqs : DRM_SCHED_PRIORITY_COUNT,
	    args->credit_limit, args->hang_limit, args->timeout,
	    args->timeout_wq, args->score, args->name, args->dev));
}
#define	drm_sched_init(sched, args)	amdxdna_drm_sched_init(sched, args)
#define	drm_sched_start(sched, err)	(drm_sched_start)(sched)
#define	drm_sched_job_init(job, ent, credits, owner, client_id)	\
	(drm_sched_job_init)(job, ent, credits, owner)

#endif
