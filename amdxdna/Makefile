.include "../kconfig.mk"

SRCDIR=	${.CURDIR:H}/drivers/accel/amdxdna

.PATH:	${SRCDIR}

KMOD=	amdxdna

SRCS=	aie.c \
	aie_psp.c \
	aie_smu.c \
	aie2_ctx.c \
	aie2_error.c \
	aie2_message.c \
	aie2_pci.c \
	aie2_pm.c \
	aie2_solver.c \
	aie4_ctx.c \
	aie4_message.c \
	aie4_pci.c \
	amdxdna_cbuf.c \
	amdxdna_ctx.c \
	amdxdna_debugfs.c \
	amdxdna_freebsd.c \
	amdxdna_gem.c \
	amdxdna_iommu.c \
	amdxdna_mailbox.c \
	amdxdna_mailbox_helper.c \
	amdxdna_pci_drv.c \
	amdxdna_pm.c \
	amdxdna_sysfs.c \
	amdxdna_ubuf.c \
	drm_gem_shmem_helper.c \
	npu1_regs.c \
	npu3_regs.c \
	npu4_regs.c \
	npu5_regs.c \
	npu6_regs.c

SRCS+=	opt_acpi.h \
	opt_drm.h \
	opt_platform.h \
	bus_if.h \
	device_if.h \
	pci_if.h \
	vnode_if.h

CLEANFILES+=	${KMOD}.ko.full ${KMOD}.ko.debug

.include <bsd.kmod.mk>

.include "../linuxkpi_version.mk"
.include "../compiler_flags.mk"

CFLAGS+=	-I${SRCDIR}/compat
CFLAGS+=	-I${.CURDIR:H}/linuxkpi/gplv2/include
CFLAGS+=	-I${.CURDIR:H}/linuxkpi/bsd/include
CFLAGS+=	-I${SYSDIR}/compat/linuxkpi/common/include
CFLAGS+=	-I${SYSDIR}/compat/linuxkpi/dummy/include
CFLAGS+=	-I${SRCDIR}
CFLAGS+=	-I${.CURDIR:H}/include
CFLAGS+=	-I${.CURDIR:H}/include/drm
CFLAGS+=	-I${.CURDIR:H}/include/uapi
CFLAGS+=	-I${.CURDIR:H}/drivers/gpu

CFLAGS+=	'-DKBUILD_MODNAME="${KMOD}"'
CFLAGS+=	'-DLINUXKPI_PARAM_PREFIX=amdxdna_' -DDRM_SYSCTL_PARAM_PREFIX=_${KMOD}
CFLAGS+=	${KCONFIG:C/(.*)/-DCONFIG_\1/}
CFLAGS+=	-include ${SRCDIR}/amdxdna_freebsd.h

CWARNFLAGS+=	-Wno-cast-qual -Wno-missing-prototypes -Wno-format -Wno-gnu-pointer-arith
CWARNFLAGS.amdxdna_mailbox.c+=	-Wno-unused-variable
