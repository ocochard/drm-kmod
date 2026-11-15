# AMD ISP 4.1 camera test driver, FreeBSD only (no LinuxKPI).

SRCDIR=	${.CURDIR:H:H}/drivers/gpu/drm/amd/amdgpu

KMOD=	amdisp
SRCS=	amdisp.c
CFLAGS+= -I${SRCDIR}

.include <bsd.kmod.mk>
