# $NetBSD$
#

DISTNAME=	OpenRTM-aist-1.1.0-RELEASE
CATEGORIES=	net
MASTER_SITES=	http://www.openrtm.org/pub/OpenRTM-aist/cxx/1.1.0/
EXTRACT_SUFX=	.tar.bz2

MAINTAINER=	n-ando@aist.go.jp
HOMEPAGE=	http://www.openrtm.org/pub/OpenRTM-aist/cxx/1.1.0/
COMMENT=	RT-Middleware and OMG RTC framework
#LICENSE=	# TODO: (see mk/license.mk)

PKG_DESTDIR_SUPPORT=	user-destdir

WRKSRC=		${WRKDIR}/OpenRTM-aist-1.1.0
GNU_CONFIGURE=	yes
USE_LIBTOOL=	yes
USE_TOOLS+=	pkg-config
USE_TOOLS+=     gmake
USE_LANGUAGES=	c c++

MAKE_ENV+=      OPSYS=${OPSYS}
.include "../../mk/bsd.prefs.mk"

PKGCONFIG_OVERRIDE+=	src/lib/coil/utils/coil-config/libcoil.pc.in
PKGCONFIG_OVERRIDE+=	utils/rtm-config/openrtm-aist.pc.in

# OpenRTM-aist requires native threads support
PTHREAD_OPTS=   require native

.if ${OPSYS} == "QNX"
LIBS+= -lsocket
LDFLAGS+= -L/lib/
LDFLAGS+= -L/usr/lib/
LDFLAGS+= -L/usr/qnx650/target/qnx6/x86/lib/gcc/4.4.2/
LDFLAGS+= -L/usr/qnx650/target/qnx6/x86/lib/
LDFLAGS+= -Wl,-L/lib/
LDFLAGS+= -Wl,-L/usr/lib/
LDFLAGS+= -Wl,-L/usr/qnx650/target/qnx6/x86/lib/gcc/4.4.2/
LDFLAGS+= -Wl,-L/usr/qnx650/target/qnx6/x86/lib/
.endif

#CONFIGURE_ARGS+= LDFLAGS="-L/usr/qnx650/target/qnx6/x86/lib/gcc/4.4.2/ -L/usr/qnx650/target/qnx6/x86/lib/"
#CONFIGURE_ARGS+= LIBS="-lsocket"

do-build:
	echo "hogedara" > /tmp/darahoge
	cd ${WRKSRC}/src/lib/coil && make
	cd ${WRKSRC} && make

.include "../omniORB416/buildlink3.mk"
.include "../../mk/pthread.buildlink3.mk"
.include "../../mk/bsd.pkg.mk"
