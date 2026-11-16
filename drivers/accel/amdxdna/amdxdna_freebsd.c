// SPDX-License-Identifier: GPL-2.0
/*
 * FreeBSD: GEM backing pages from a LinuxKPI shmem object.  drm-kmod
 * builds drm_gem_get_pages() and drm_gem_put_pages() on Linux only.
 */
#include <linux/mm.h>
#include <linux/shmem_fs.h>
#include <linux/slab.h>
#include <drm/drm_gem.h>

struct page **
amdxdna_drm_gem_get_pages(struct drm_gem_object *obj)
{
	struct page **pages, *page;
	long i, npages;

	if (WARN_ON(obj->filp == NULL))
		return (ERR_PTR(-EINVAL));
	WARN_ON((obj->size & (PAGE_SIZE - 1)) != 0);

	npages = obj->size >> PAGE_SHIFT;
	pages = kvmalloc_array(npages, sizeof(*pages), GFP_KERNEL);
	if (pages == NULL)
		return (ERR_PTR(-ENOMEM));

	for (i = 0; i < npages; i++) {
		/* Returns a wired page; put_page() unwires it. */
		page = shmem_read_mapping_page(obj->filp->f_shmem, i);
		if (IS_ERR(page)) {
			while (--i >= 0)
				put_page(pages[i]);
			kvfree(pages);
			return (ERR_CAST(page));
		}
		pages[i] = page;
	}

	return (pages);
}

void
amdxdna_drm_gem_put_pages(struct drm_gem_object *obj, struct page **pages,
    bool dirty, bool accessed)
{
	long i, npages;

	npages = obj->size >> PAGE_SHIFT;
	for (i = 0; i < npages; i++) {
		if (pages[i] == NULL)
			continue;
		if (dirty)
			set_page_dirty(pages[i]);
		put_page(pages[i]);
	}
	kvfree(pages);
}

/*
 * Carveout memory.  Without IOMMU SVA, amdxdna needs a physically
 * contiguous region whose addresses the NPU uses directly.  Linux takes
 * one reserved at boot (memmap=) through debugfs; FreeBSD allocates it
 * at probe, sized by the hw.amdxdna.carveout_mb tunable.  Allocate as
 * early as possible: a large contiguous run gets hard to find later.
 *
 * The pages are wired and belong to no VM object, like TTM pages:
 * lkpi_vmf_insert_pfn_prot_locked() moves a page into the mapping's
 * object, which must not happen to pages of the kernel object.  The
 * kernel reaches them through the direct map (memremap()).
 */
#include <sys/param.h>
#include <vm/vm.h>
#include <vm/vm_page.h>

#include "amdxdna_pci_drv.h"
#include "amdxdna_cbuf.h"

static unsigned int carveout_mb;
module_param(carveout_mb, uint, 0400);
MODULE_PARM_DESC(carveout_mb, "Carveout size in MB, 0 disables (FreeBSD)");

int
amdxdna_freebsd_carveout_init(struct amdxdna_dev *xdna)
{
	u_long i, npages;
	vm_page_t m;
	int ret;

	if (carveout_mb == 0)
		return (0);

	npages = ((u_long)carveout_mb << 20) >> PAGE_SHIFT;
	m = vm_page_alloc_noobj_contig(VM_ALLOC_WIRED | VM_ALLOC_ZERO,
	    npages, 0, ~(vm_paddr_t)0, 2 << 20, 0, VM_MEMATTR_DEFAULT);
	if (m == NULL) {
		XDNA_ERR(xdna, "Cannot allocate %u MB contiguous carveout",
		    carveout_mb);
		return (-ENOMEM);
	}

	ret = amdxdna_carveout_init(xdna, VM_PAGE_TO_PHYS(m),
	    (u64)npages << PAGE_SHIFT);
	if (ret != 0) {
		for (i = 0; i < npages; i++) {
			vm_page_unwire_noq(&m[i]);
			vm_page_free(&m[i]);
		}
		return (ret);
	}
	xdna->carveout_pages = m;
	xdna->carveout_npages = npages;
	return (0);
}

void
amdxdna_freebsd_carveout_fini(struct amdxdna_dev *xdna)
{
	vm_page_t m = xdna->carveout_pages;
	u_long i;

	if (m == NULL)
		return;
	for (i = 0; i < xdna->carveout_npages; i++) {
		vm_page_unwire_noq(&m[i]);
		vm_page_free(&m[i]);
	}
	xdna->carveout_pages = NULL;
}

/*
 * User address of a BO mapping.  LinuxKPI calls the driver mmap with
 * vm_start 0, before the address is chosen, so amdxdna records 0 for a
 * mapped carveout BO.  Find the real address in the calling process:
 * the map entry backed by the device pager object whose handle is the
 * dma-buf private data given to the pager at mmap time.  LinuxKPI maps
 * that object from offset 0, the start of the BO.
 */
#include <sys/proc.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <linux/dma-buf.h>

#include "amdxdna_gem.h"

u64
amdxdna_freebsd_gem_uva(struct amdxdna_gem_obj *abo)
{
	struct vm_map_entry *entry;
	vm_object_t obj;
	vm_map_t map;
	void *handle;
	u64 uva;

	if (abo->type == AMDXDNA_BO_DEV || abo->mem.uva != 0 ||
	    !is_import_bo(abo))
		return (abo->mem.uva);

	handle = abo->attach->dmabuf->priv;
	uva = AMDXDNA_INVALID_ADDR;
	map = &curproc->p_vmspace->vm_map;
	vm_map_lock_read(map);
	VM_MAP_ENTRY_FOREACH(entry, map) {
		if ((entry->eflags & MAP_ENTRY_IS_SUB_MAP) != 0)
			continue;
		obj = entry->object.vm_object;
		if (obj == NULL || obj->handle != handle ||
		    (obj->type != OBJT_MGTDEVICE && obj->type != OBJT_DEVICE))
			continue;
		uva = entry->start - entry->offset;
		break;
	}
	vm_map_unlock_read(map);
	return (uva);
}
