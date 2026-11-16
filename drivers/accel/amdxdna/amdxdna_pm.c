// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_drv.h>
#include <linux/pm_runtime.h>

#include "amdxdna_pm.h"

#define AMDXDNA_AUTOSUSPEND_DELAY	5000 /* milliseconds */

int amdxdna_pm_suspend(struct device *dev)
{
	struct amdxdna_dev *xdna = to_xdna_dev(dev_get_drvdata(dev));
	int ret = -EOPNOTSUPP;

	guard(mutex)(&xdna->dev_lock);
	if (xdna->dev_info->ops->suspend)
		ret = xdna->dev_info->ops->suspend(xdna);

	XDNA_DBG(xdna, "Suspend done ret %d", ret);
	return ret;
}

int amdxdna_pm_resume(struct device *dev)
{
	struct amdxdna_dev *xdna = to_xdna_dev(dev_get_drvdata(dev));
	int ret = -EOPNOTSUPP;

	guard(mutex)(&xdna->dev_lock);
	if (xdna->dev_info->ops->resume)
		ret = xdna->dev_info->ops->resume(xdna);

	XDNA_DBG(xdna, "Resume done ret %d", ret);
	return ret;
}

#ifdef __linux__
int amdxdna_pm_resume_get(struct amdxdna_dev *xdna)
{
	struct device *dev = xdna->ddev.dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		XDNA_ERR(xdna, "Resume failed: %d", ret);
		pm_runtime_set_suspended(dev);
	}

	return ret;
}

void amdxdna_pm_suspend_put(struct amdxdna_dev *xdna)
{
	struct device *dev = xdna->ddev.dev;

	pm_runtime_put_autosuspend(dev);
}

void amdxdna_pm_init(struct amdxdna_dev *xdna)
{
	struct device *dev = xdna->ddev.dev;

	pm_runtime_set_active(dev);
	pm_runtime_set_autosuspend_delay(dev, AMDXDNA_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_allow(dev);
	pm_runtime_put_autosuspend(dev);
}

void amdxdna_pm_fini(struct amdxdna_dev *xdna)
{
	struct device *dev = xdna->ddev.dev;

	pm_runtime_get_noresume(dev);
	pm_runtime_forbid(dev);
}
#else /* !__linux__ */
/*
 * XXX FreeBSD: LinuxKPI has no runtime PM core.  Count the users the
 * driver declares with amdxdna_pm_resume_get() and _suspend_put(), and
 * AMDXDNA_AUTOSUSPEND_DELAY after the last one stop the NPU and put it in
 * D3, as the Linux PCI runtime PM does.  Lock order: pm_lock, then
 * dev_lock (taken by amdxdna_pm_suspend() and _resume()).
 */
static void amdxdna_pm_work(struct work_struct *work)
{
	struct amdxdna_dev *xdna = container_of(to_delayed_work(work),
	    struct amdxdna_dev, pm_work);
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);

	mutex_lock(&xdna->pm_lock);
	if (atomic_read(&xdna->pm_usage) == 0 && !xdna->pm_suspended &&
	    amdxdna_pm_suspend(xdna->ddev.dev) == 0) {
		pci_save_state(pdev);
		pci_set_powerstate(pdev->dev.bsddev, PCI_POWERSTATE_D3);
		xdna->pm_suspended = true;
	}
	mutex_unlock(&xdna->pm_lock);
}

int amdxdna_pm_resume_get(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret = 0;

	mutex_lock(&xdna->pm_lock);
	if (xdna->pm_suspended) {
		/* Back to D0, then the configuration space. */
		pci_restore_state(pdev);
		ret = amdxdna_pm_resume(xdna->ddev.dev);
		if (ret)
			XDNA_ERR(xdna, "Resume failed: %d", ret);
		else
			xdna->pm_suspended = false;
	}
	if (!ret)
		atomic_inc(&xdna->pm_usage);
	mutex_unlock(&xdna->pm_lock);

	return ret;
}

void amdxdna_pm_suspend_put(struct amdxdna_dev *xdna)
{
	/* Called with dev_lock held: no pm_lock here. */
	if (atomic_dec_return(&xdna->pm_usage) == 0)
		mod_delayed_work(system_wq, &xdna->pm_work,
		    msecs_to_jiffies(AMDXDNA_AUTOSUSPEND_DELAY));
}

void amdxdna_pm_setup(struct amdxdna_dev *xdna)
{
	mutex_init(&xdna->pm_lock);
	atomic_set(&xdna->pm_usage, 0);
	xdna->pm_suspended = false;
	INIT_DELAYED_WORK(&xdna->pm_work, amdxdna_pm_work);
}

void amdxdna_pm_init(struct amdxdna_dev *xdna)
{
	mod_delayed_work(system_wq, &xdna->pm_work,
	    msecs_to_jiffies(AMDXDNA_AUTOSUSPEND_DELAY));
}

/* Before the device teardown, without dev_lock: wake the NPU, keep it on. */
void amdxdna_pm_quiesce(struct amdxdna_dev *xdna)
{
	cancel_delayed_work_sync(&xdna->pm_work);
	if (amdxdna_pm_resume_get(xdna))
		XDNA_WARN(xdna, "Teardown with the NPU suspended");
}

void amdxdna_pm_fini(struct amdxdna_dev *xdna)
{
	/* pm_usage > 0 since amdxdna_pm_quiesce(): the work cannot suspend. */
	cancel_delayed_work_sync(&xdna->pm_work);
}

/*
 * System sleep.  As the Linux PCI core does, wake a runtime-suspended NPU
 * before its suspend op: resuming a firmware that was stopped before
 * the system suspend leaves the NPU unable to power on until a reboot.
 * The reference held across the sleep keeps the work from suspending it.
 */
int amdxdna_pm_sys_suspend(struct device *dev)
{
	struct amdxdna_dev *xdna = to_xdna_dev(dev_get_drvdata(dev));
	int ret;

	cancel_delayed_work_sync(&xdna->pm_work);
	ret = amdxdna_pm_resume_get(xdna);
	if (ret)
		return ret;
	return amdxdna_pm_suspend(dev);
}

int amdxdna_pm_sys_resume(struct device *dev)
{
	struct amdxdna_dev *xdna = to_xdna_dev(dev_get_drvdata(dev));
	int ret;

	ret = amdxdna_pm_resume(dev);
	amdxdna_pm_suspend_put(xdna);
	return ret;
}
#endif /* __linux__ */
