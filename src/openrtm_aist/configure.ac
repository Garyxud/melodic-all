dnl -*- autoconf -*-
dnl
dnl @file configure.ac
dnl @brief Input file for autoconf
dnl @date $Date: 2008-02-26 15:21:04 $
dnl @author Noriaki Ando <n-ando@aist.go.jp>
dnl
dnl This file is an input file used by the GNU "autoconf" program to
dnl generate the file "configure", which tries to guess your system
dnl configuration
dnl
dnl
dnl $Id$
dnl

dnl ------------------------------------------------------------
dnl  Autoconf initial macros
dnl ------------------------------------------------------------
dnl  Process this file with autoconf to produce a configure script.
AC_PREREQ(2.59)
AC_INIT(OpenRTM-aist, 1.1.2, n-ando@aist.go.jp, OpenRTM-aist)
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([dist-bzip2 tar-pax])
AM_CONFIG_HEADER([src/lib/rtm/config_rtc.h])

dnl ------------------------------------------------------------
dnl Config files created at configure
dnl ------------------------------------------------------------
dnl pkgconfig pc file
AC_CONFIG_FILES([utils/rtm-config/openrtm-aist.pc:utils/rtm-config/openrtm-aist.pc.in])
AC_CONFIG_FILES([utils/rtm-config/rtm-config:utils/rtm-config/rtm-config.in])

dnl RPM package spec file and build script
dnl AC_CONFIG_FILES([packages/rpm/OpenRTM-aist.spec:packages/rpm/OpenRTM-aist.spec.in])
dnl AC_CONFIG_FILES([packages/rpm/rpm_build.sh:packages/rpm/rpm_build.sh.in])

dnl ------------------------------------------------------------
dnl rename config header's PACKAGE_* definition
dnl ------------------------------------------------------------
mv src/lib/rtm/config_rtc.h.in src/lib/rtm/config_rtc.h.in.org
sed -e 's/undef PACKAGE/undef RTM_PACKAGE/g' src/lib/rtm/config_rtc.h.in.org | sed -e 's/\#define VERSION .*//g' > src/lib/rtm/config_rtc.h.in
rm -f src/lib/rtm/config_rtc.h.in.org

AC_CONFIG_SUBDIRS([src/lib/coil])

dnl ------------------------------------------------------------
dnl  Version number
dnl ------------------------------------------------------------
major_version=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\1/g'`]
minor_version=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\2/g'`]
revision_num=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\3/g'`]
short_version=$major_version$minor_version$revision_num

RTM_VERSION=$VERSION
RTM_MAJOR_VERSION=$major_version
RTM_MINOR_VERSION=$minor_version
RTM_REVISION_NUM=$revision_num
RTM_SHORT_VERSION=$short_version
RTM_DESCRIPTION='An OMG RTC compliant middleware for robots.'

AC_DEFINE_UNQUOTED([RTM_VERSION],       "$VERSION",       [RTM version])
AC_DEFINE_UNQUOTED([RTM_MAJOR_VERSION], $major_version,   [RTM major version])
AC_DEFINE_UNQUOTED([RTM_MINOR_VERSION], $minor_version,   [RTM minor version])
AC_DEFINE_UNQUOTED([RTM_REVISION_NUM],  $revision_num,    [RTM revision numver])
AC_DEFINE_UNQUOTED([RTM_SHORT_VERSION], "$short_version", [RTM short version])

dnl ------------------------------------------------------------
dnl  Basic setup
dnl ------------------------------------------------------------
AC_PROG_CXX
AC_PROG_CXXCPP
AC_LANG_CPLUSPLUS
AC_PROG_INSTALL
AC_LIBTOOL_WIN32_DLL
AC_PROG_LIBTOOL
AM_WITH_DMALLOC
AC_CHECK_HEADERS([nlist.h stdlib.h unistd.h])
AC_CHECK_FUNCS([uname])

CPPFLAGS="-Wall -fPIC $CPPFLAGS"
CXXFLAGS="-Wall -fPIC $CXXFLAGS"

AC_SEARCH_LIBS(pthread_create, pthread c_r, , 
	AC_MSG_ERROR([** thread library is needed **])) 

SHLIB_SUFFIX=".so"

dnl ------------------------------------------------------------
dnl  configure arguments definition
dnl ------------------------------------------------------------

dnl  ACE directory
AC_ARG_WITH(pkgconfigdir,
	AC_HELP_STRING([--with-pkgconfigdir],
	[Use the specified pkgconfig dir (default is libdir/pkgconfig]),
	[pkgconfigdir=${withval}],
	[pkgconfigdir='${libdir}/pkgconfig'])
AC_MSG_NOTICE([pkgconfig directory is ${pkgconfigdir}])
AC_ARG_WITH(ace-includes,     [  --with-ace-includes=dir         Find ACE include files installation below dir(without "include")])
AC_ARG_WITH(ace-lib,          [  --with-ace-lib=dir              Find ACE lib files installation below dir(without "lib"])

dnl  boost directory
#AC_ARG_WITH(boost-includes,   [  --with-boost-includes=dir       Find boost include files installation below dir])
#AC_ARG_WITH(boost-lib,        [  --with-boost-lib=dir            Find boost lib files installation below dir])

dnl  ORB selection
AC_ARG_WITH(mico,             [  --with-mico=dir                 Find MICO installation below dir])
AC_ARG_WITH(orbix,            [  --with-orbix=dir                Find Orbix installation below dir])
AC_ARG_WITH(orbacus,          [  --with-orbacus=dir              Find ORBacus installation below dir])
AC_ARG_WITH(tao,              [  --with-tao=dir                  Find TAO installation below dir])
AC_ARG_WITH(omniorb,          [  --with-omniorb=dir              Find omniORB installation below dir])
AC_ARG_WITH(orbit2,           [  --with-orbit2=dir               Find ORBit2 installation below dir])
AC_ARG_WITH(orbexpress,       [  --with-orbexpress=dir           Find ORBexpress installation below dir])
AC_ARG_WITH(rtorb,            [  --with-rtorb=dir                Find RtORB installation below dir])
AC_ARG_WITH(generic-orb,      [  --with-generic-orb              Use other CORBA 2.3 ORB])
AC_ARG_WITH(generic-orb-lib,  [  --with-generic-orb-lib	=libs    ORB libraries (-l...)])
AC_ARG_WITH(doxygen,          [  --without-doxygen               Don't create documentation by doxygen])

dnl  Script wrapper selection
#AC_ARG_WITH(python,           [  --with-python=dir               Find Python installation below dir])
#AC_ARG_WITH(ruby,             [  --with-ruby=dir                 Find Ruby installation below dir])
#AC_ARG_WITH(perl,             [  --with-perl=dir                 Find Perl installation below dir])
#AC_ARG_WITH(tcl,              [  --with-tcl=dir                  Find Tcl installation below dir])
#AC_ARG_WITH(euslisp,          [  --with-euslisp=dir              Find EusLisp installation below dir])

dnl Additional compile options

AC_ARG_ENABLE(debug,          [  --enable-debug                  Compile using -g])
AC_ARG_ENABLE(optimize,       [  --disable-optimize              Don't compile using -O])
AC_ARG_WITH(includes,         [  --with-includes=dirs            Additional include dirs (separated by ':')])
AC_ARG_WITH(libs,             [  --with-libs=dirs                Additional lib dirs (separated by ':')])
AC_ARG_ENABLE(gcc2option,     [  --with-gcc2                     Compile with gcc2])
AC_ARG_ENABLE(artlinux,       [  --with-artlinux                 With ART-Linux])
AC_ARG_ENABLE(sockdataport,   [  --with-sockdataport             With Socket DataPort])
AC_ARG_ENABLE(unittest,       [  --enable-test                   Enable UnitTest With CppUnit])




dnl ------------------------------------------------------------
dnl  Determine system name
dnl ------------------------------------------------------------

AC_MSG_CHECKING([system name])
system=`uname -s`-`uname -r`
if test "`uname -s`" = "AIX" ; then
	system=AIX-`uname -v`.`uname -r`
fi

case $system in
[FreeBSD*)]
	AC_DEFINE([RTM_OS_FREEBSD], [TRUE], [OS is FreeBSD])
	LDFLAGS="-export-dynamic $LDFLAGS"
	os="freebsd"
	;;	
[Linux*)]
	AC_DEFINE([RTM_OS_LINUX], [TRUE], [OS is Linux])
	AC_DEFINE([_REENTRANT], [TRUE], [Multi Thread Support])
	LDFLAGS="-luuid -ldl -export-dynamic $LDFLAGS"
	os="linux"
	;;	
[HP-UX*)]
	AC_DEFINE([RTM_OS_HPUX], [TRUE], [OS is HP-UX])
	os="hp-ux"
	;;	
[SunOS*)]
	AC_DEFINE([RTM_OS_SUNOS], [TRUE], [OS is SunOS])
	os="sunos"
	;;	
[IRIX*)]
	AC_DEFINE([RTM_OS_IRIX], [TRUE], [OS is IRIX])
	os="irix"
	;;	
[OSF1*)]
	AC_DEFINE([RTM_OS_OSF1], [TRUE], [OS is OSF1])
	os="osf1"
	;;	
[Darwin*)]
	AC_DEFINE([RTM_OS_DARWIN], [TRUE], [OS is Max OS X])
	SHLIB_SUFFIX=".dylib"
	LDFLAGS="-export-dynamic $LDFLAGS"
	os="darwin"
	;;	
[QNX*)]
	AC_DEFINE([RTM_OS_QNX], [TRUE], [OS is QNX])
	os="qnx"
	;;	
[CYGWIN*)]
	AC_DEFINE([RTM_OS_CYGWIN], [TRUE], [OS is Cygwin])
	os="cygwin"
	CPPFLAGS="-DCygwin $CPPFLAGS"
	CXXFLAGS="-DCygwin $CXXFLAGS"
	LDFLAGS="-no-undefined -luuid -ldl -export-dynamic $LDFLAGS"
	;;
*)
	echo "Unkonown system"
	;;
esac

AC_MSG_RESULT($system)

AC_MSG_CHECKING(compiler flags)
[CXXFLAGS=`echo $CXXFLAGS | sed 's/-g //g' | sed 's/-O[0-9]*//g'`]
if test "x$enable_debug" = "xyes" ; then
	CXXFLAGS="$CXXFLAGS -g"
elif test "x$enable_optimize" != "xno" ; then
	CXXFLAGS="$CXXFLAGS -O2"
fi
if test "x$CXXFLAGS" = "x" ; then
        AC_MSG_RESULT(none)
else
        AC_MSG_RESULT($CXXFLAGS)
fi

if test "x$with_gcc2" = "xyes" ; then
	CXXFLAGS="$CXXFLAGS -ftemplate-depth-30"
	AC_DEFINE([RTM_GCC2], [TRUE], [RTM is compiled with gcc2])
fi



dnl ------------------------------------------------------------
dnl  Prepare for setup
dnl ------------------------------------------------------------
orb_to_use=
more_incs="`echo $with_includes | sed 's/:/ /g'`"
more_libs="`echo $with_libs     | sed 's/:/ /g'`"
base_incs="`echo $CPLUS_INCLUDE_PATH | sed 's/:/ /g'` /local/include /usr/local/include /usr/include"
base_libs="`echo $LIBPATH $LIBRARY_PATH $LD_LIBRARY_PATH $SHLIB_PATH | sed 's/:/ /g'` /usr/local/lib /usr/lib /usr/lib64"
base_bins="`echo $PATH | sed 's/:/\/.. /g'` /local/bin /usr/local/bin /usr/bin"
base_dirs="$prefix `echo $PATH | sed 's/:/\/.. /g'`"


dnl ------------------------------------------------------------
dnl  Use with_includes and with_libs
dnl ------------------------------------------------------------
for ipath in $more_incs ; do
	CPPFLAGS="-I$ipath $CPPFLAGS"
	CXXFLAGS="-I$ipath $CXXFLAGS"
done

for lpath in $more_libs ; do
	LDFLAGS="-L$lpath $LDFLAGS"
done



dnl this is needed for creating libraries using libtool
dnl AC_LIBTOOL_WIN32_DLL
dnl AC_LIBLTDL_CONVENIENCE
dnl AC_LIBLTDL_INSTALLABLE
dnl AC_LIBTOOL_DLOPEN

dnl AC_CONFIG_SUBDIRS(libltdl)
dnl AC_LIB_LTDL
dnl AC_SUBST(INCLTDL)
dnl AC_SUBST(LIBLTDL)

dnl Checks for libraries.
dnl FIXME: Replace `main' with a function in `-le':
dnl AC_CHECK_LIB([e], [main])
dnl FIXME: Replace `main' with a function in `-links':
dnl AC_CHECK_LIB([inks], [main])

dnl Checks for header files.


dnl Checks for typedefs, structures, and compiler characteristics.
dnl AC_C_CONST
dnl AC_TYPE_PID_T
dnl AC_STRUCT_TM

dnl Checks for library functions.
dnl AC_FUNC_ERROR_AT_LINE


dnl ============================================================
dnl Check Doxygen setup
dnl
dnl 1. check if doxygen installed
dnl 2. set DOCS variable
dnl ============================================================
dnl ------------------------------------------------------------
dnl  Find doxygen
dnl ------------------------------------------------------------
DOCS="docs"
if test "x$with_doxygen" = "xno" ; then
	DOCS=""
else
	AC_MSG_CHECKING(for doxygen)
	FP_FIND_FILE(doxygen,$base_bins $base_dirs,bin)
	if test "x$doxygen" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(Please install doxygen or use --without-doxygen option)
	else
		AC_MSG_RESULT($doxygen)
	fi
	DOCS="docs"
fi

dnl ============================================================
dnl Check boost setup
dnl
dnl 1. check boost/version.hpp for Boost
dnl 2. check Boost version from boost/version.hpp
dnl 3. check libboost_regex.so
dnl ============================================================
dnl ------------------------------------------------------------
dnl  Find boost/version.hpp
dnl ------------------------------------------------------------
dnl AC_MSG_CHECKING(for boost/version.hpp)
dnl FP_FIND_FILE(boost/version.hpp,
dnl 	$with_boost_includes $more_incs $base_incs $base_dirs,
dnl 	include cxx/include)
dnl if test "x$boost_version_hpp_dir" = "x" ; then
dnl 	AC_MSG_RESULT(Boost (boost/version.hpp) not found)
dnl 	AC_MSG_ERROR(
dnl use --with-boost-includes=<dir> to locate Boost's headers.
dnl hint: <dir>/( include cxx/include ) will be searched.)
dnl 	# Error exit!!
dnl else
dnl 	AC_MSG_RESULT(found in $boost_version_hpp_dir)
dnl 	have_boost=yes
dnl fi
dnl 
dnl ------------------------------------------------------------
dnl  Check boost version/Find libboost_regex.so
dnl ------------------------------------------------------------
dnl if test "x$have_boost" = "xyes" ; then
dnl 	AC_MSG_CHECKING(Boost version)
dnl 	boost_version=[`grep ' BOOST_VERSION ' $boost_version_hpp_dir/boost/version.hpp | grep '#define' | cut -d  ' ' -f 3`]
dnl 
dnl 	case $boost_version in
dnl 	[1?????)]
dnl 		;;
dnl 	*)
dnl 		AC_MSG_WARN([Unknown Boost version number, expecting 1.3.0 or above.])
dnl 	esac
dnl  	AC_MSG_RESULT($boost_version)
dnl 
dnl 	# Checking libboost_regex.so
dnl 	AC_MSG_CHECKING(for libboost_regex.so)
dnl 	FP_FIND_FILE(libboost_regex.so,
dnl 		$with_boost_lib $more_libs $base_libs $base_dirs,
dnl 		lib cxx/lib)
dnl 	if test "x$libboost_regex_so_dir" = "x" ; then
dnl 		AC_MSG_RESULT(not found)
dnl 		AC_MSG_ERROR(
dnl 	use --with-boost-lib=<dir> to locate libboost_regex.so
dnl hint: <dir>/( lib cxx/lib ) will be searched.)
dnl 		# Error exit!!
dnl 	else
dnl 		AC_MSG_RESULT(found in $libboost_regex_so_dir)
dnl 	fi
dnl 
dnl 	#
dnl 	# Boost setup complete
dnl 	#
dnl 	# Compiler/Linker options
dnl 	CPPFLAGS="$CPPFLAGS -I$boost_version_hpp_dir"
dnl 	CXXFLAGS="$CXXFLAGS -I$boost_version_hpp_dir"
dnl 	LDFLAGS="$LDFLAGS -L$libboost_regex_so_dir "
dnl 	LIBS="$LIBS -lboost_regex"
dnl 	LDSOLIBS="$LDSOLIBS -lboost_regex"
dnl 	AC_DEFINE([WITH_BOOST], [TRUE], [compiled with Boost])
dnl fi



dnl ============================================================
dnl ORB configuration
dnl
dnl ============================================================
if test "x$with_mico" != "x" ; then
	orb_to_use=MICO
fi
if test "x$with_orbix" != "x" ; then
	orb_to_use=Orbix
fi
if test "x$with_orbacus" != "x" ; then
	orb_to_use=ORBacus
fi
if test "x$with_omniorb" != "x" ; then
	orb_to_use=omniORB
fi
if test "x$with_tao" != "x" ; then
	orb_to_use=TAO
fi
if test "x$with_orbit2" != "x" ; then
	orb_to_use=ORBit2
fi
if test "x$with_orbexpress" != "x" ; then
	orb_to_use=ORBexpress
fi
if test "x$with_rtorb" != "x" ; then
   	orb_to_use=RtORB
fi
if test "x$orb_to_use" = "x" ; then
	orb_to_use=omniORB
fi

echo 'ORB: '$orb_to_use
echo 'OpenRTM-aist will be compiled with '$orb_to_use


dnl ============================================================
dnl Check Mico setup
dnl
dnl 1. check mico-setup.sh getting MICO setup information
dnl 2. obtain some VARIABLES from mico-setup.sh 
dnl 3. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_mico=no

if test "x$orb_to_use" = "xMICO" ; then
	dnl ------------------------------------------------------------
	dnl  Find mico-setup.sh
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for mico-setup.sh)
	if test "x$MICOVERSION" != "x" && test "x$MICODIR" != "x" ; then
		AC_MSG_RESULT(already loaded)
		have_mico=yes
	else
		FP_FIND_FILE(mico-setup.sh,
			$with_mico $more_libs $base_libs $base_dirs,
			lib)
		if test "x$mico_setup_sh" = "x" ; then
			AC_MSG_RESULT(not found)
			AC_MSG_ERROR(
	use --with-mico=<dir> to locate MICO setup directories.
hint: <dir>/lib will be searched.)
			# Error exit!!
		else
			AC_MSG_RESULT($mico_setup_sh)
			AC_MSG_CHECKING(contents of MICO config)
			#
			# mico_config unsets prefix and exec_prefix
			#
			save_prefix="$prefix"
			save_exec_prefix="$exec_prefix"
			. $mico_setup_sh
			prefix="$save_prefix"
			exec_prefix="$save_exec_prefix"
			AC_MSG_RESULT(done)
			have_mico=yes
		fi
	fi
fi

dnl ------------------------------------------------------------
dnl  Check mico version
dnl ------------------------------------------------------------
if test "x$have_mico" = "xyes" ; then
	AC_MSG_CHECKING(MICO version)
	AC_MSG_RESULT($MICOVERSION)

	case $MICOVERSION in
	[2.[012].*|2.3.[0123])]
		AC_MSG_WARN([MICO version unusable, need at least 2.3.4])
		have_mico=no
		;;
	[2.[3-9].*)]
		;;
	*)
		AC_MSG_WARN([Unknown MICO version number, expecting 2.3.0 or above.])
		AC_MSG_WARN([If it doesn't work, use --disable-mico])
		;;
	esac
fi
dnl ------------------------------------------------------------
dnl  Check CORBA.h path, libmico.so
dnl ------------------------------------------------------------
if test "x$have_mico" = "xyes" ; then
	AC_MSG_CHECKING(for CORBA.h)
	FP_FIND_FILE(CORBA.h,
		$MICODIR $MICOSHARED $with_mico $more_incs $base_incs $base_dirs,
		include cxx/include)
	if test "x$CORBA_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(
use --with-mico=<dir> to locate mico's CORBA.h
hint: <dir>/( include cxx/include ) will be searched.)
		# Error exit!!
	else
		AC_MSG_RESULT(found in $CORBA_h_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Check libmico.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libmico${MICOVERSION}${SHLIB_SUFFIX})
	FP_FIND_FILE(libmico${MICOVERSION}${SHLIB_SUFFIX},
		$MICODIR $MICOSHAREDDIR $with_mico $more_libs $base_libs $base_dirs,
		lib, mico_lib)
	if test "x$mico_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_CHECKING(for libmico${MICOVERSION}.a)
		FP_FIND_FILE(libmico${MICOVERSION}.a,
			$MICODIR MICOSHAREDDIR $with_mico $more_libs $base_libs $base_dirs,
			lib, mico_lib)
	fi
	if test "x$mico_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)		
		AC_MSG_ERROR(
use --with-mico=<dir> to locate libmico${MICOVERSION}.so or libmico${MICOVERSION}.a
hint: <dir>/lib will be searched.)
		# Error exit!!
	else
		AC_MSG_RESULT(found in $mico_lib_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Check whether we must use MiniSTL
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(whether we must use MiniSTL)
	cat > conftest.c <<EOF
#include <CORBA.h>
#ifdef HAVE_MINI_STL
MiniSTL:yes
#else
MiniSTL:no
#endif
EOF
	have_mini_stl=[`${CXX-c++} -I$CORBA_h_dir -E conftest.c 2> /dev/null | grep '^MiniSTL' | cut -d: -f 2`]
	rm -f conftest.*
	if test "x$have_mini_stl" != "xyes" && test "x$have_mini_stl" != "xno" ; then
		have_mini_stl=unknown
	fi
	AC_MSG_RESULT($have_mini_stl)
	if test "x$have_mini_stl" = "xyes" ; then
		CPPFLAGS="-I$CORBA_h_dir/ministl $CPPFLAGS"
	fi

	dnl ------------------------------------------------------------
	dnl  Check mico-ld
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for miFPco-ld)
	FP_FIND_FILE(mico-ld,
		$MICODIR $MICOSHAREDDIR $with_mico $base_bins $base_dirs,
		bin)
	if test "x$mico_ld" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(
use --with-mico=<dir> to locate mico-ld.
hint: <dir>/bin will be searched.)
		# Error exit!!
	else
		AC_MSG_RESULT($mico_ld)
	fi

	dnl ------------------------------------------------------------
	dnl  Check MICO dependencies
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(MICO dependencies)
	micodeps=
	for item in `grep -- -l $mico_ld | tr '"' ' '` ; do # "
		case $item in
		-l*) micodeps="$micodeps $item" ;;
		esac
	done
	AC_MSG_RESULT($micodeps)

	dnl ------------------------------------------------------------
	dnl  Check MICO idl
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for idl)
	FP_FIND_FILE(idl,
		$MICODIR $MICOSHAREDDIR $with_mico $base_bins $base_dirs,
		bin)
	if test "x$idl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(
use --with-mico=<dir> to locate idl.
hint: <dir>/bin will be searched.)
		# Error exit!!
	else
		AC_MSG_RESULT($idl)
	fi

	dnl ------------------------------------------------------------
	dnl  Mico setup complete
	dnl ------------------------------------------------------------
	PATH_NORM($idl)
	IDLC=$n_path
	IDL_FLAGS="--c++-skel --poa --no-boa --any"
	CXXFLAGS="-I$CORBA_h_dir $CXXFLAGS"
	LDFLAGS="$LDFLAGS -L$mico_lib_dir"
	LIBS="$LIBS -lmico$MICOVERSION -lmicocoss$MICOVERSION $micodeps"
	LDSOLIBS="$LDSOLIBS -lmico$MICOVERSION -lmicocoss$MICOVERSION $micodeps"

	if test "x$micoir_lib_dir" != "x" ; then
		LIBS="-lmicoir$MICOVERSION -lmicocoss$MICOVERSION $LIBS"
		LDSOLIBS="-lmicoir$MICOVERSION $LDSOLIBS"
	fi
	AC_DEFINE([ORB_IS_MICO], [TRUE], [ORB is MICO])
fi

if test "x$orb_to_use" = "x" && test "x$have_mico" = "xyes" ; then
	orb_to_use=MICO
fi

dnl end of mico test
dnl ------------------------------------------------------------


dnl ============================================================
dnl Check ORBacus setup
dnl
dnl 1. check OB/CORBA.h for ORBacus
dnl 2. check ORBacus version
dnl 3. check ORBacus library "libOB.so, libJTC.so"
dnl 4. check ORBacus idl compiler
dnl 5. check ORBacus irserv
dnl 4. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_orbacus=no

if test "x$orb_to_use" = "xORBacus" ; then
	AC_MSG_CHECKING(for OB/CORBA.h)
	FP_FIND_FILE(OB/CORBA.h,$with_orbacus $more_incs $base_incs $base_dirs,include)
	if test "x$OB_CORBA_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $OB_CORBA_h_dir)
		have_orbacus=yes
	fi
fi

if test "x$have_orbacus" = "xyes" ; then
	AC_MSG_CHECKING(ORBacus version)
	cat > conftest.c <<EOF
#include <OB/CORBA.h>
version:OB_VERSION
EOF
	ob_version=[`${CXX-c++} -I$OB_CORBA_h_dir -E conftest.c 2> /dev/null | grep ^version | cut -d: -f 2 | sed 's/[" ]//g'`]
	# "
	rm -f conftest.*
	if test "x$ob_version" = "x" ; then
		ob_version=unknown
	fi
	AC_MSG_RESULT($ob_version)
	case $ob_version in
	3*|4.0a|4.0b1|4.0b2*)
		AC_MSG_WARN([ORBacus $ob_version unusable, need at least 4.0b3])
		have_orbacus=no
		;;
	4.0b*|4.0|4.0.*)
		;;
	4.1|4.1.*)
		;;
	*)
		AC_MSG_WARN([Unknown ORBacus version number, expecting 4.0b3 or above.])
		AC_MSG_WARN([If it doesn't work, use --disable-orbacus])
		;;
	esac
fi

if test "x$have_orbacus" = "xyes" ; then
	AC_MSG_CHECKING(for libOB.so)
	FP_FIND_FILE(libOB.so,$with_orbacus $more_libs $base_libs $base_dirs,lib,ob_lib)
	if test "x$ob_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_CHECKING(for libOB.a)
		FP_FIND_FILE(libOB.a,$with_orbacus $more_libs $base_libs $base_dirs,lib,ob_lib)
	fi
	if test "x$ob_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbacus to locate libOB.so or libOB.a)
	else
		AC_MSG_RESULT(found in $ob_lib_dir)
	fi

	AC_MSG_CHECKING(for libJTC.so)
	FP_FIND_FILE(libJTC.so,$with_orbacus $more_libs $base_libs $base_dirs,lib,ob_jtc_lib)
	if test "x$ob_jtc_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_CHECKING(for libJTC.a)
		FP_FIND_FILE(libJTC.a,$with_orbacus $more_libs $base_libs $base_dirs,lib,ob_jtc_lib)
	fi
	if test "x$ob_jtc_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)

		case $ob_version in
		4.0*)
			AC_MSG_WARN(assuming single-threaded ORBacus)
			;;
		4.1|4.1.*)
			AC_MSG_ERROR(use --with-orbacus to locate libJTC.so or libJTC.a)
			;;
		esac
	else
		AC_MSG_RESULT(found in $ob_jtc_lib_dir)
	fi

	if test "x$enable_orbacus_repo" != "xno" ; then
		AC_MSG_RESULT(enable_orbacus_repo)
#		AC_DEFINE(COMBAT_ORBACUS_LOCAL_REPO)
	fi

	AC_MSG_CHECKING(for idl)
	FP_FIND_FILE(idl,$with_orbacus $base_bins $base_dirs,bin)
	if test "x$idl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbacus to locate idl)
	else
		AC_MSG_RESULT($idl)

	fi

	dnl ------------------------------------------------------------
	dnl  ORBacus setup complete
	dnl ------------------------------------------------------------
	PATH_NORM($idl)
	IDLC = $n_path
	IDL_FLAGS="--c-suffix .cc"
	IFRFEED="$irfeed"
	IFRSERVER="$irserv"

	# Compiler/Linker options
	# CPPFLAGS="$CPPFLAGS -I$OB_CORBA_h_dir"
	CXXFLAGS="-I$OB_CORBA_h_dir $CXXFLAGS"
	LDFLAGS="-L$ob_lib_dir $LDFLAGS"

	if test "x$ob_jtc_lib_dir" != "x" ; then
		LIBS=" $LIBS -lJTC"
		LDSOLIBS="$LIBS -lJTC"
	fi

	LIBS="$LIBS -lOB"
	LDSOLIBS="$LDSOLIBS -lOB"
	AC_DEFINE([ORB_IS_ORBACUS], [TRUE], [ORB is ORBacus])
fi

if test "x$orb_to_use" = "x" && test "x$have_orbacus" = "xyes" ; then
	orb_to_use=ORBacus
fi

dnl end of ORBacus setup
dnl ------------------------------------------------------------


dnl ------------------------------------------------------------
dnl Check Orbix/E setup
dnl
dnl 1. check CORBA.hh for Orbix
dnl 2. check Orbix library "libit_art.so"
dnl 3. check Orbix version using "idl"
dnl 4. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ------------------------------------------------------------
have_orbix=no

if test "x$orb_to_use" = "xOrbix" ; then
	dnl ------------------------------------------------------------
	dnl  find Orbix/E's corba.hh
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for OBE/CORBA.h)
	# orbix/corba.hh -> $orbix_corba_hh, $orbix_corba_hh_dir 
	FP_FIND_FILE(OBE/CORBA.h,
		$with_orbix $ART_INSTALL_PATH $more_incs $base_incs $base_dirs,
		include cxx/include)
	if test "x$obe_corba_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $obe_corba_h_dir)
		have_orbix=yes
	fi
fi

if test "x$have_orbix" = "xyes" ; then
	dnl ------------------------------------------------------------
	dnl  Find libit_art.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libobea_server.so)
	FP_FIND_FILE(libobea_server.so,$with_orbix $ART_INSTALL_PATH $more_libs $base_libs $base_dirs,lib cxx/lib)
	if test "x$libobea_server_so_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbix to locate libit_art.so)
	else
		AC_MSG_RESULT(found in $libobea_server_so_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Find idl
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for aeidl)
	FP_FIND_FILE(aeidl,$with_orbix $ART_INSTALL_PATH $base_bins $base_dirs,bin)
	if test "x$aeidl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbix to locate idl)
	else
		AC_MSG_RESULT($aeidl)
	fi

	dnl ------------------------------------------------------------
	dnl  Find enameserv
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for enameserv)
	FP_FIND_FILE(idl,$with_orbix $ART_INSTALL_PATH $base_bins $base_dirs,bin)
	if test "x$idl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbix to locate idl)
	else
		AC_MSG_RESULT($enameserv)
	fi

	dnl ------------------------------------------------------------
	dnl  Check Orbix/E's version
	dnl ------------------------------------------------------------	
	AC_MSG_CHECKING(OrbixE version)
	orbix_version=[`$idl -V 2>&1 | grep Version | sed 's/^.*Version //'`]
	if test "x$orbix_version" = "x" ; then
		orbix_version=unknown
	fi
	AC_MSG_RESULT($orbix_version)

	dnl ------------------------------------------------------------
	dnl  Orbix/E setup complete
	dnl ------------------------------------------------------------
	PATH_NORM($aeidl)
	IDLC = $n_path
	IDL_FLAGS="-base:-ccc:-hh -poa:-ccc:-hh"
	COSNAMES = $enameserv
	dnl  Compiler/Linker options
	dnl  CPPFLAGS="$CPPFLAGS -D_REENTRANT -I$orbix_corba_hh_dir"
	CXXFLAGS="-D_REENTRANT -I$orbix_corba_hh_dir $CXXFLAGS"
	LDFLAGS="$LDFLAGS -L$libit_art_so_dir"
	LIBS="$LIBS -lit_dynany -lit_poa -lit_art -lit_itl -lit_ts -lit_base -lpthread"
	LDSOLIBS="$LDSOLIBS -lit_dynany -lit_poa -lit_art -lit_itl -lit_ts -lit_base -lpthread"
	AC_DEFINE([ORB_IS_ORBIX], [TRUE], [ORB is Orbix])
fi

if test "x$orb_to_use" = "x" && test "x$have_orbix" = "xyes" ; then
	orb_to_use=OrbixE
fi


dnl ============================================================
dnl TAO setup
dnl
dnl 1. check tao/corba.h for TAO
dnl 2. check TAO library "libTAO.so"
dnl 3. check TAO idl compiler "tao_idl"
dnl 4. check TAO version using tao_idl
dnl 5. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_tao=no

if test "x$orb_to_use" = "xTAO" ; then
	dnl ------------------------------------------------------------
	dnl  find TAO's corba.h
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for tao/corba.h)
	dnl  tao/corba.h -> $tao_corba_h, $tao_corba_h_dir 
	FP_FIND_FILE(tao/corba.h,
		$with_tao $TAO_ROOT $ACE_ROOT/tao/include $more_incs $base_incs $base_dirs,
		include cxx/include)
	if test "x$tao_corba_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $tao_corba_h_dir)
		tao_include="-I$tao_corba_h_dir"
		have_tao=yes
	fi
fi

if test "x$have_tao" = "xyes" ; then
	dnl ------------------------------------------------------------
	dnl  Find libTAO.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libTAO.so)
	dnl  libTAO.so -> %libTAO_so $libTAO_so_dir
	FP_FIND_FILE(libTAO.so,$with_tao $more_libs $base_libs $base_dirs,lib cxx/lib)
	if test "x$libTAO_so_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-tao to locate libTAO.so)
	else
		AC_MSG_RESULT(found in $libTAO_so_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Find tao_idl
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for tao_idl)
	dnl  tao_idl -> $tao_idl $tao_idl_dir
	FP_FIND_FILE(tao_idl,$with_tao $base_bins $base_dirs,bin)
	if test "x$tao_idl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-tao to locate tao_idl)
	else
		AC_MSG_RESULT($tao_idl)

	fi

	dnl ------------------------------------------------------------
	dnl  Find TAO's Naming_Service
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for Naming_Service)
	dnl  tao_naming_service -> $tao_naming_service $tao_naming_service_dir
	FP_FIND_FILE(Naming_Service,$with_tao $base_bins $base_dirs,bin)
	if test "x$Naming_Service" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-tao to locate tao_idl)
	else
		AC_MSG_RESULT($Naming_Service)

	fi

	dnl ------------------------------------------------------------
	dnl  Check TAO's version
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(TAO version)
	tao_version=[`$tao_idl -V 2>&1 | grep 'TAO' | grep 'version' | sed 's/^.*version //'`]
	if test "x$tao_version" = "x" ; then
		tao_version=unknown
	fi
	AC_DEFINE([RTC_CORBA_CXXMAPPING11], [TRUE], [CORBA C++ mapping is ver1.1])
	AC_MSG_RESULT($tao_version)

	dnl ------------------------------------------------------------
	dnl  TAO setup complete
	dnl ------------------------------------------------------------
	PATH_NORM($tao_idl)
	IDLC=$n_path
	IDL_FLAGS="-DTAO_IDL -Ge 0 $tao_include/tao $tao_include"
	COSNAMES=$Naming_Service
	CPPFLAGS="$tao_include $CPPFLAGS"
	CXXFLAGS="$tao_include $CXXFLAGS"
	LDFLAGS="-L$libTAO_so_dir $LDFLAGS"
	LIBS="-lTAO -lTAO_CosNaming -lTAO_DynamicAny -lTAO_DynamicInterface $LIBS "
	LDSOLIBS="-lTAO -lTAO_CosNaming -lTAO_DynamicAny -lTAO_DynamicInterface $LDSOLIBS"
	AC_DEFINE([ORB_IS_TAO], [TRUE], [ORB is TAO])
fi

if test "x$orb_to_use" = "x" && test "x$have_tao" = "xyes" ; then
	orb_to_use=TAO
fi

dnl end of TAO setup
dnl ------------------------------------------------------------


dnl ============================================================
dnl omniORB setup
dnl
dnl 1. check omniORB4/CORBA.h for omniORB
dnl 2. check omniORB library "libomniORB4.so"
dnl 3. check omniORB idl compiler "omniidl"
dnl 4. check omniORB version using libomniORB4.so
dnl 5. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_omniorb=no

if test "x$orb_to_use" = "xomniORB" ; then
	dnl ------------------------------------------------------------
	dnl  find omniORB's CORBA.h
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for omniORB4/CORBA.h)
	dnl  omniORB/CORBA.h -> $omniORB4_CORBA_h, $omniORB4_CORBA_h_dir 
	FP_FIND_FILE(omniORB4/CORBA.h,
		$with_omniorb/include $more_incs $base_incs $base_dirs,
		include cxx/include)
	if test "x$omniORB4_CORBA_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $omniORB4_CORBA_h_dir)
		have_omniorb=yes
	fi
fi

if test "x$have_omniorb" = "xyes" ; then
	dnl ------------------------------------------------------------
	dnl  Find libomniORB4.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libomniORB4.so)
	dnl  libomniORB4.so -> %libomniORB4_so $libomniORB4_so_dir
	FP_FIND_FILE(libomniORB4${SHLIB_SUFFIX},$with_omniorb/lib $more_libs $base_libs $base_dirs,lib cxx/lib, omniorb_lib)
	if test "x$omniorb_lib_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-omniorb to locate libomniORB4.so)
	else
		AC_MSG_RESULT(found in $omniorb_lib_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Find omniidl
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for omniidl)
	dnl  omniidl -> $omniidl $omniidl_dir
	FP_FIND_FILE(omniidl,$with_omniorb/bin $base_bins $base_dirs,bin)
	if test "x$omniidl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-omniorb to locate omniidl)
	else
		AC_MSG_RESULT($omniidl)

	fi

	dnl ------------------------------------------------------------
	dnl  Find omniNames
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for omniNames)
	dnl  omninames -> $omninames $omninames_dir
	FP_FIND_FILE(omniNames,$with_omniorb/bin $base_bins $base_dirs,bin)
	if test "x$omniNames" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-omniorb to locate omniNames)
	else
		AC_MSG_RESULT($omniNames)
	fi

	dnl ------------------------------------------------------------
	dnl  Check omniORB version
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(omniORB version)
	omniorb_version=[`strings $omniorb_lib_dir/libomniORB4${SHLIB_SUFFIX} 2>&1 | grep 'omniORB version' | sed 's/^.*version //'`]
	if test "x$omniorb_version" = "x" ; then
		omniorb_version=unknown
	fi
	case $omniorb_version in
	4.0.*)
		AC_MSG_RESULT(CORBA C++ mapping ver1.0 is used.)
		AC_DEFINE([RTM_OMNIORB_40], [TRUE], [omniORB version is 4.0])
		AC_MSG_RESULT(omniORB version is 4.0.)
		;;
	4.1|4.1.*)
		AC_DEFINE([RTC_CORBA_CXXMAPPING11], [TRUE], [CORBA C++ mapping is ver1.1])
		AC_MSG_RESULT(CORBA C++ mapping ver1.1 is used.)
		AC_DEFINE([RTM_OMNIORB_41], [TRUE], [omniORB version is 4.1])
		AC_MSG_RESULT(omniORB version is 4.1.)
		;;
	4.2|4.2.*)
		AC_DEFINE([RTC_CORBA_CXXMAPPING11], [TRUE], [CORBA C++ mapping is ver1.1])
		AC_MSG_RESULT(CORBA C++ mapping ver1.1 is used.)
		AC_DEFINE([RTM_OMNIORB_42], [TRUE], [omniORB version is 4.2])
		AC_MSG_RESULT(omniORB version is 4.2.)
		;;
	esac
	AC_MSG_RESULT($omnirob_version)


	dnl ------------------------------------------------------------
	dnl  omniORB setup complete
	dnl ------------------------------------------------------------
	dnl omniidl and commandline options
	PATH_NORM($omniidl)
	omniidl=$n_path
	IDLC=$omniidl
	IDL_FLAGS="-bcxx -Wba -nf -Wbuse_quotes"

	dnl omniNames
	PATH_NORM($omniNames)
	omniNames=$n_path
	COSNAMES=$omniNames

	dnl  Compiler/Linker options
	CPPFLAGS="-I$omniORB4_CORBA_h_dir $CPPFLAGS"
	CXXFLAGS="-I$omniORB4_CORBA_h_dir $CXXFLAGS"
	LDFLAGS="-L$omniorb_lib_dir $LDFLAGS"
	LIBS="$LIBS -lomniORB4 -lomnithread -lomniDynamic4"
	LDSOLIBS="$LDSOLIBS -lomniORB4 -lomnithread -lomniDynamic4"
	AC_DEFINE([ORB_IS_OMNIORB], [TRUE], [ORB is omniORB])
fi

if test "x$orb_to_use" = "x" && test "x$have_omniorb" = "xyes" ; then
	orb_to_use=omniORB
fi

dnl end of omniORB setup
dnl------------------------------------------------------------

dnl ============================================================
dnl ORBit2 setup
dnl
dnl 1. check orbit2-config
dnl 2. check libORBit-2-cpp.so
dnl 3. check orbit-idl-2
dnl 4. check omniORB version using libomniORB4.so
dnl 5. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_orbit2=no

if test "x$orb_to_use" = "xORBit2" ; then
	dnl ------------------------------------------------------------
	dnl  find ORBit2's orbit2-config script and check version
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for orbit2-config)
	orbit2_version=`orbit2-config --version | sed -e 's/ORBit2 //g'`
	if test "x$orbit2_version" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found. ORBit2 version $orbit2_version.)
		have_orbit2=yes
		orbit2_prefix=`orbit2-config --prefix`
		orbit2_exec_prefix=`orbit2-config --exec-prefix`
		orbit2_libs=`orbit2-config --libs --use-service=name`
		orbit2_cflags=`orbit2-config --cflags --use-service=name`
	fi
fi
	

if test "x$have_orbit2" = "xyes" ; then
	dnl ------------------------------------------------------------
	dnl  Find libORBit-2-cpp.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libORBit-2-cpp.so)
	dnl  libORBit-2-cpp.so -> $libORBit_2_so $libORBit_2_cpp_so_dir
	FP_FIND_FILE(libORBit-2-cpp.so,$with_orbit2 $orbit2_prefix $more_libs $base_libs $base_dirs,lib cxx/lib)
	if test "x$libORBit_2_cpp_so_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbit2=dir to locate <dir>/lib/libORBit-2-cpp.so)
	else
		AC_MSG_RESULT(found in $libORBit_2_cpp_so_dir)
	fi

	dnl ------------------------------------------------------------
	dnl  Find orbit-idl-2
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for orbit-idl-2)
	dnl  orbit-id_2 -> $orbit_idl_2 $orbit_idl_2_dir
	FP_FIND_FILE(orbit-idl-2,$with_orbit2 $orbit2_exec_prefix $more_libs $base_dirs,bin)
	if test "x$orbit_idl_2" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-orbit2=dir to locate bin prefix. ex. <dir>/bin/orbit-idl-2)
	else
		AC_MSG_RESULT(found. $orbit_idl_2)
	fi

	dnl ------------------------------------------------------------
	dnl  ORBit2 setup complete
	dnl ------------------------------------------------------------
	PATH_NORM($orbit_idl_2)
	IDLC=$n_path
	IDL_FLAGS="--lang cpp -DORBIT2_IDL"
	COSNAMES=
	dnl  Compiler/Linker options
	CPPFLAGS="$orbit2_cflags -I$orbit2_prefix/include/orbitcpp-2.0 $CPPFLAGS"
	CXXFLAGS="$orbit2_cflags -I$orbit2_prefix/include/orbitcpp-2.0 $CXXFLAGS"
	LDFLAGS="$LDFLAGS $orbit2_libs"
	LIBS="$LIBS $orbit2_libs -lORBit-2-cpp -lORBitCosNaming-2-cpp"
	LDSOLIBS="$LDSOLIBS $orbit2_libs -lORBit-2-cpp -lORBitCosNaming-2-cpp"
	AC_DEFINE([ORB_IS_ORBIT2], [TRUE], [ORB is ORBit2])
fi

if test "x$orb_to_use" = "x" && test "x$have_orbit2" = "xyes" ; then
	orb_to_use=ORBit2
fi

dnl end of ORBit2 setup
dnl------------------------------------------------------------

dnl ------------------------------------------------------------
dnl ORBexpress
dnl ------------------------------------------------------------

dnl end of ORBexpres
dnl ------------------------------------------------------------

dnl ============================================================
dnl RtORB setup
dnl
dnl 1. check RtORB/corba.hh for RtORB
dnl 2. check RtORB library "libRtORB.so and libRtORB_cpp.so"
dnl 3. check RtORB idl compiler "rtorb-idl"
dnl 4. set IDL compile options, CPPFLAGS, LDFLAGS, LIBS etc...
dnl ============================================================
have_rtorb=no

if test "x$orb_to_use" = "xRtORB" ; then
	dnl ------------------------------------------------------------
	dnl  find ORBit2's orbit2-config script and check version
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for RtORB/corba.hh)
	FP_FIND_FILE(RtORB/corba.hh,
			$with_rtorb/include $more_incs $base_incs $base_dirs,
			include cxx/include)
	if test "x$RtORB_corba_hh_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-rtorb to locate RtORB)
	else
		AC_MSG_RESULT(found in $RtORB_corba_hh_dir)
		have_rtorb=yes
	fi
fi
	
if test "x$have_rtorb" = "xyes" ; then
	dnl ------------------------------------------------------------
	dnl  Find libRtORB.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libRtORB.so)
	if test "x$os" = "xcygwin" ; then
		FP_FIND_FILE(RtORB.dll,
			$with_rtorb $more_libs $base_libs $base_dirs,lib cxx/lib)
		if test "x$RtORB_dll_dir" = "x" ; then
			AC_MSG_RESULT(not found)
			AC_MSG_ERROR(use --with-rtorb=dir to locate <dir>/lib/RtORB.dll)
		else
			AC_MSG_RESULT(found in $RtORB_dll_dir)
			dnl shared lib dir is unified to "$libRtORB_so_dir"
			libRtORB_so_dir=$RtORB_dll_dir
		fi
	else
		FP_FIND_FILE(libRtORB.so,
			$with_rtorb/lib $more_libs $base_libs $base_dirs,
			lib cxx/lib)
		if test "x$libRtORB_so_dir" = "x" ; then
			AC_MSG_RESULT(not found)
			AC_MSG_ERROR(use --with-rtorb=dir to locate <dir>/lib/libRtORB.so)
		else
			AC_MSG_RESULT(found in $libRtORB_so_dir)
		fi
	fi

	dnl ------------------------------------------------------------
	dnl  Find libRtORB_cpp.so
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for libRtORB_cpp.so)
	if test "x$os" = "xcygwin" ; then
		FP_FIND_FILE(RtORB_cpp.dll,
			$with_rtorb/lib $more_libs $base_libs $base_dirs,
			lib cxx/lib)
		if test "x$RtORB_cpp_dll_dir" = "x" ; then
			AC_MSG_RESULT(not found)
			AC_MSG_ERROR(use --with-rtorb=dir to locate <dir>/lib/RtORB_cpp.dll)
		else
			AC_MSG_RESULT(found in $RtORB_cpp_dll_dir)
			dnl shared lib dir is unified to "$libRtORB_cpp_so_dir"
			libRtORB_cpp_so_dir=$RtORB_cpp_dll_dir
		fi
	else
		FP_FIND_FILE(libRtORB_cpp.so,
			$with_rtorb/lib $more_libs $base_libs $base_dirs,
			lib cxx/lib)
		if test "x$libRtORB_cpp_so_dir" = "x" ; then
			AC_MSG_RESULT(not found)
			AC_MSG_ERROR(use --with-rtorb=dir to locate <dir>/lib/libRtORB_cpp.so)
		else
			AC_MSG_RESULT(found in $libRtORB_cpp_so_dir)
		fi
	fi

	dnl ------------------------------------------------------------
	dnl  Find rtorb-idl
	dnl ------------------------------------------------------------
	AC_MSG_CHECKING(for rtorb-idl)
	FP_FIND_FILE(rtorb-idl,$with_rtorb/bin $base_bins $base_dirs,bin)
	if test "x$rtorb_idl" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(use --with-rtorb=dir to locate bin prefix. ex. <dir>/bin/rtorb-idl)
	else
		AC_MSG_RESULT(found. $rtorb_idl)
	fi

	dnl ------------------------------------------------------------
	dnl  RtORB setup complete
	dnl ------------------------------------------------------------
	IDLC=$rtorb_idl	
	IDL_FLAGS="--skeleton-impl -DORBIT2_IDL -I."
	COSNAMES=$with_rtorb/CosName/NameService
	dnl  Compiler/Linker options
	CPPFLAGS="$CPPFLAGS -I$RtORB_corba_hh_dir"
	CXXFLAGS="$CXXFLAGS -I$RtORB_corba_hh_dir"
	LDFLAGS="$LDFLAGS -L$libRtORB_cpp_so_dir -L$libRtORB_so_dir"

	if test "x$os" = "xcygwin" ; then 
		LIBS="$LIBS -lRtORB -lRtORB_cpp -lpthread"
		LDSOLIBS="$LDSOLIBS -lRtORB -lRtORB_cpp -lpthread"
	else
		LIBS="$LIBS -lRtORB -lRtORB_cpp -lpthread"
		LDSOLIBS="$LDSOLIBS -lRtORB -lRtORB_cpp -lpthread"
	fi
	AC_DEFINE([ORB_IS_RTORB], [TRUE], [ORB is RtORB])
fi

if test "x$orb_to_use" = "x" && test "x$have_rtorb" = "xyes" ; then
	orb_to_use=RtORB
fi

dnl end of RtORB setup
dnl------------------------------------------------------------

dnl ------------------------------------------------------------
dnl  ORB setup
dnl ------------------------------------------------------------
AC_MSG_CHECKING(ORB to use)
case $orb_to_use in
MICO)
        AC_MSG_RESULT([MICO, version $MICOVERSION])
        ;;
OrbixE)
        AC_MSG_RESULT([OrbixE, version $orbix_version])
        ;;
ORBacus)
        AC_MSG_RESULT([ORBacus, version $ob_version])
        ;;
TAO)
        AC_MSG_RESULT([TAO, version $tao_version])
        ;;
omniORB)
        AC_MSG_RESULT([omniORB, version $omniorb_version])
        ;;
ORBit2)
        AC_MSG_RESULT([ORBit2, version $orbit2_version])
        ;;
ORBexpress)
        AC_MSG_RESULT([ORBexpress, version $orbexpress_version])
        ;;
RtORB)
        AC_MSG_RESULT([RtORB, version $rtorb_version])
        ;;
generic)
        AC_MSG_RESULT([generic ORB, unknown version])
        ORB_SOURCES="event-corba.cc"
        AC_MSG_WARN([Attempting to compile with generic ORB configuration.])
        AC_MSG_WARN([You may still have to tweak the Makefile. Good luck.])
        ;;
*)
        AC_MSG_RESULT(none)
        AC_MSG_ERROR([no ORB found, check ./configure --help])
        ;;
esac

ORB=$orb_to_use

AM_CONDITIONAL(ORB_IS_TAO, test x$orb_to_use = xTAO)
AM_CONDITIONAL(ORB_IS_ORBIX, test x$orb_to_use = xOrbix)
AM_CONDITIONAL(ORB_IS_ORBACUS, test x$orb_to_use = xORBacus)
AM_CONDITIONAL(ORB_IS_OMNIORB, test x$orb_to_use = xomniORB)
AM_CONDITIONAL(ORB_IS_MICO, test x$orb_to_use = xMICO)
AM_CONDITIONAL(ORB_IS_ORBIT2, test x$orb_to_use = xORBit2)
AM_CONDITIONAL(ORB_IS_ORBEXPRESS, test x$orb_to_use = xORBexpress)
AM_CONDITIONAL(ORB_IS_RTORB, test x$orb_to_use = xRtORB)


dnl ------------------------------------------------------------
dnl Script wrapper code generation
dnl ------------------------------------------------------------
WRAPPERS=


dnl ------------------------------------------------------------
dnl Python setup
dnl
dnl 1. check Python version
dnl 2. check Python header "Python.h"
dnl ------------------------------------------------------------
have_python=no

if test "x$with_python" != "x" ; then
	AC_MSG_CHECKING(Python version)

	python_version=[`python -V 2>&1 |  sed -e 's/Python //g'`]
	case $python_version in
	[2.3.*|2.4.*|2.5.*)]
		;;
	*)
		AC_MSG_WARN([Unknown Python version number, expecting 2.3.0 or above.])
	esac
 	AC_MSG_RESULT($python_version)

	dnl  get majour.minor version number without releasse level number
	python_version=[`echo $python_version | awk 'BEGIN{FS="\."}{printf("%d.%d\n",$1,$2)}'`]
	dnl  find Python's Python.h
	dnl  python2.3/Python.h -> $Python_h_dir
	AC_MSG_CHECKING(for python/Python.h)
	FP_FIND_FILE(Python.h,
		$with_python $more_incs $base_incs $base_dirs /usr/local/include/python$python_version /usr/include/python$python_version,
		include cxx/include)
	if test "x$Python_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $Python_h_dir)
		have_python=yes
	fi
fi

if test "x$have_python" = "xyes" ; then
	PYTHON_FLAGS="-I$Python_h_dir -D__PTHREAD_ -D__THREAD__"
	WRAPPERS="$WRAPEERS python"
	AC_DEFINE([PYTHON_WRAPPER], [TRUE], [Python wrapper is enable])
dnl 	echo 'Python wrapper enabled'
dnl 	echo 'PYTHON_INCLUDES: '$PYTHON_FLAGS
fi

dnl end of Python setup
dnl------------------------------------------------------------

dnl ------------------------------------------------------------
dnl ARTLinux setup
dnl
dnl ------------------------------------------------------------
if test "x$os" = "xlinux" ; then
   	RTPREEMPTEC="rtpreempt"
	AC_MSG_CHECKING(for art_task.h)
	FP_FIND_FILE(art_task.h,
		$with_artlinux $more_incs $base_incs $base_dirs /usr/src/linux/include,
		linux)
	if test "x$art_task_h_dir" = "x" ; then
		AC_MSG_RESULT(not found)
	else
		AC_MSG_RESULT(found in $art_task_h_dir)
	fi

	if test "x$art_task_h_dir" != "x" ; then
		AC_MSG_CHECKING(for art_syscalls.o)
		FP_FIND_FILE(art_syscalls.o,
			$with_artlinux $more_incs $base_incs $base_dirs /usr/src/linux/arch/i386,
			lib)
		if test "x$art_syscalls_o_dir" = "x" ; then
			AC_MSG_RESULT(not found)
		else
			AC_MSG_RESULT(found in $art_syscalls_o_dir)
			have_artlinux=yes
		fi
		AC_DEFINE([ARTLINUX], [TRUE], [ARTLinux is avaiable])
		ARTLINUX="artlinux"
		ARTLIB="$art_syscalls_o_dir/art_syscalls.o"
		echo 'RTM is compiled with ARTLinux'
	fi
fi

dnl ------------------------------------------------------------
dnl RDTSC (Read Time Stamp Counter) for measure high resolution time
dnl
dnl ------------------------------------------------------------

if test "x$with_rdtsc" != "x" ; then
	AC_DEFINE([RTM_RTDSC], [TRUE], [RDTSC is enable])
	echo 'RTM is compiled with RDTSC'
	echo 'Warning!! System architecture should be Intel!!'
fi


dnl ------------------------------------------------------------
dnl Socket DataPort
dnl
dnl ------------------------------------------------------------
if test "x$with_sockdataport" != "x" ; then
	if test "x$orb_to_use" = "xomniORB" ; then
		AC_DEFINE([RTC_SOCK_DATAPORT], [TRUE], [Socket DataPort is enable])
	else
		AC_MSG_WARN([To use socket dataport, CORBA should be omniORB.])
	fi
fi


dnl ------------------------------------------------------------
dnl Unit test
dnl
dnl ------------------------------------------------------------
if test "x$enable_test" = "xyes" ; then
	UNITTEST="tests"
fi



dnl ------------------------------------------------------------
dnl exported environmental variables
AC_SUBST(TARGET)
AC_SUBST(LIBRARY)
AC_SUBST(CC)
AC_SUBST(CXX)
AC_SUBST(CFLAGS)
AC_SUBST(CPPFLAGS)
AC_SUBST(CXXFLAGS)
dnl AC_SUBST(BUILD_CFLAGS)
dnl AC_SUBST(BUILD_CPPFLAGS)
AC_SUBST(LD)
AC_SUBST(LDFLAGS)
AC_SUBST(LIBS)
AC_SUBST(LDSO)
AC_SUBST(SOEXT)
AC_SUBST(LDSOOPTS)
AC_SUBST(LDSOLIBS)
AC_SUBST(pkgconfigdir)

dnl RTM version related options
AC_SUBST(RTM_VERSION)
AC_SUBST(RTM_MAJOR_VERSION)
AC_SUBST(RTM_MINOR_VERSION)
AC_SUBST(RTM_REVISION_NUM)
AC_SUBST(RTM_SHORT_VERSION)
AC_SUBST(RTM_DESCRIPTION)

dnl ORB related options
AC_SUBST(ORB)
AC_SUBST(IDLC)
AC_SUBST(IDL_FLAGS)
AC_SUBST(COSNAMES)

dnl Documentation related options
AC_SUBST(DOCS)

dnl Wrapper code subdirectory 
AC_SUBST(WRAPPERS)
AC_SUBST(PYTHON_FLAGS)
dnl ARTLinux lib
AC_SUBST(ARTLINUX)
AC_SUBST(ARTLIB)
AC_SUBST(UNITTEST)

dnl RT preempt EC
AC_SUBST(RTPREEMPTEC)

dnl OpenRTM's includedir/libdir/datadir definition
rtm_includedir=$includedir"/openrtm-"$major_version"."$minor_version
rtm_idldir=$rtm_includedir"/rtm/idl"
rtm_libdir=$libdir"/openrtm-"$major_version"."$minor_version
rtm_datadir=$datadir"/openrtm-"$major_version"."$minor_version
coil_includedir=$includedir"/coil-"$major_version"."$minor_version

dnl under lib/openrtm-x.y directories
rtm_rtcdir=$rtm_libdir"/rtc"
rtm_ecdir=$rtm_libdir"/ec"
rtm_rtmdir=$rtm_libdir"/rtm"
rtm_svcdir=$rtm_libdir"/svc"
rtm_sdodir=$rtm_libdir"/sdo"
rtm_docdir=$rtm_datadir"/doc"
rtm_exampledir=$rtm_datadir"/example"

AC_SUBST(rtm_libdir)
AC_SUBST(rtm_datadir)
AC_SUBST(rtm_includedir)
AC_SUBST(rtm_idldir)
AC_SUBST(coil_includedir)
AC_SUBST(rtm_rtcdir)
AC_SUBST(rtm_ecdir)
AC_SUBST(rtm_rtmdir)
AC_SUBST(rtm_svcdir)
AC_SUBST(rtm_sdodir)
AC_SUBST(rtm_docdir)
AC_SUBST(rtm_exampledir)

dnl ------------------------------------------------------------

UNIQUE($CPPFLAGS)
CPPFLAGS=$u_list" "
CPPFLAGS=`echo $CPPFLAGS | sed -e 's/-I\/usr\/include //g'`
CPPFLAGS=`echo $CPPFLAGS | sed -e 's/-I\/usr\/include$//g'`
CPPFLAGS="$more_incs $CPPFLAGS"

UNIQUE($CXXFLAGS)
CXXFLAGS=$u_list" "
CXXFLAGS=`echo $CXXFLAGS | sed -e 's/-I\/usr\/include //g'`
CPPFLAGS=`echo $CXXFLAGS | sed -e 's/-I\/usr\/include$//g'`
CXXFLAGS="$more_incs $CXXFLAGS"

UNIQUE($LDFLAGS)
LDFLAGS=$u_list" "
LDFLAGS=`echo $LDFLAGS | sed -e 's/-L\/usr\/lib //g'`
LDFLAGS=`echo $LDFLAGS | sed -e 's/-L\/usr\/lib$//g'`
LDFLAGS="$more_libs $LDFLAGS"

UNIQUE($LDSOLIBS)
LDSOLIBS=$u_list" "
LDSOLIBS=`echo $LDSOLIBS | sed -e 's/-L\/usr\/lib //g'`
LDSOLIBS=`echo $LDSOLIBS | sed -e 's/-L\/usr\/lib$//g'`
LDSOLIBS="$more_libs $LDSOLIBS"

UNIQUE($IDL_FLAGS)
IDL_FLAGS=$u_list

echo ''
echo '------------------------------------------------------------'
echo ''
echo ' '$PACKAGE_NAME' will be build with the following options. '
echo ''
echo '------------------------------------------------------------'
echo 'CXX: '$CXX
echo 'CPPFLAGS: '$CPPFLAGS
echo 'CXXFLAGS: '$CXXFLAGS
echo ''
echo 'LD: '$LD
echo 'LIBS: '$LIBS
echo 'LDFLAGS: '$LDFLAGS
echo 'LDSOLIBS: '$LDSOLIBS
echo ''
echo 'ORB: '$ORB
echo 'IDLC: '$IDLC
echo 'IDL_FLAGS: '$IDL_FLAGS
echo ''
echo 'include dir: '$rtm_includedir
echo 'lib dir:     '$rtm_libdir
echo 'data dir:    '$rtm_datadir
echo '------------------------------------------------------------'
echo ''


AC_OUTPUT([Makefile
	build/Makefile
	packages/Makefile
	packages/deb/Makefile
	packages/deb/debian/Makefile
	packages/rpm/Makefile
	src/Makefile
	src/lib/Makefile
	src/lib/rtm/Makefile
	src/lib/rtm/idl/Makefile
	src/lib/rtm/ext/Makefile
	src/lib/rtm/idl/device_interfaces/Makefile
	src/lib/rtm/tests/Makefile
	src/lib/rtm/tests/CORBA_IORUtil/Makefile
	src/lib/rtm/tests/CORBA_SeqUtil/Makefile
	src/lib/rtm/tests/ConfigAdmin/Makefile
	src/lib/rtm/tests/ConnectorListener/Makefile
	src/lib/rtm/tests/CorbaConsumer/Makefile
	src/lib/rtm/tests/CorbaNaming/Makefile
	src/lib/rtm/tests/CorbaPort/Makefile
	src/lib/rtm/tests/DataInOutPort/Makefile
	src/lib/rtm/tests/ECFactory/Makefile
	src/lib/rtm/tests/ExtTrigExecutionContext/Makefile
	src/lib/rtm/tests/Factory/Makefile
	src/lib/rtm/tests/InPort/Makefile
	src/lib/rtm/tests/InPortBase/Makefile
	src/lib/rtm/tests/InPortCorbaCdrConsumer/Makefile
	src/lib/rtm/tests/InPortCorbaCdrProvider/Makefile
	src/lib/rtm/tests/InPortPushConnector/Makefile
	src/lib/rtm/tests/InPortPullConnector/Makefile
	src/lib/rtm/tests/InPortConnector/Makefile
	src/lib/rtm/tests/InPortProvider/Makefile
	src/lib/rtm/tests/Manager/Makefile
	src/lib/rtm/tests/ManagerConfig/Makefile
	src/lib/rtm/tests/ManagerServant/Makefile
	src/lib/rtm/tests/ModuleManager/Makefile
	src/lib/rtm/tests/NVUtil/Makefile
	src/lib/rtm/tests/NamingManager/Makefile
	src/lib/rtm/tests/NumberingPolicy/Makefile
	src/lib/rtm/tests/ObjectManager/Makefile
	src/lib/rtm/tests/OutPort/Makefile
	src/lib/rtm/tests/OutPortBase/Makefile
	src/lib/rtm/tests/OutPortConnector/Makefile
	src/lib/rtm/tests/OutPortPushConnector/Makefile
	src/lib/rtm/tests/OutPortPullConnector/Makefile
	src/lib/rtm/tests/OutPortCorbaCdrConsumer/Makefile
	src/lib/rtm/tests/OutPortCorbaCdrProvider/Makefile
	src/lib/rtm/tests/OutPortProvider/Makefile
	src/lib/rtm/tests/PeriodicECSharedComposite/Makefile
	src/lib/rtm/tests/PeriodicExecutionContext/Makefile
	src/lib/rtm/tests/PortAdmin/Makefile
	src/lib/rtm/tests/PortBase/Makefile
	src/lib/rtm/tests/PublisherFlush/Makefile
	src/lib/rtm/tests/PublisherNew/Makefile
	src/lib/rtm/tests/PublisherPeriodic/Makefile
	src/lib/rtm/tests/RTCUtil/Makefile
	src/lib/rtm/tests/RTObject/Makefile
	src/lib/rtm/tests/RingBuffer/Makefile
	src/lib/rtm/tests/SdoConfiguration/Makefile
	src/lib/rtm/tests/SdoOrganization/Makefile
	src/lib/rtm/tests/SdoService/Makefile
	src/lib/rtm/tests/StateMachine/Makefile
	src/lib/rtm/tests/SystemLogger/Makefile
	src/ext/Makefile
	src/ext/ec/Makefile
	src/ext/ec/artlinux/Makefile
	src/ext/ec/rtpreempt/Makefile
	src/ext/sdo/Makefile
	src/ext/sdo/observer/Makefile
	utils/Makefile
	utils/rtm-config/Makefile
	utils/cmake/Makefile
	utils/rtc-template/Makefile
	utils/rtm-skelwrapper/Makefile
	utils/rtm-naming/Makefile
	utils/rtcd/Makefile
	utils/rtcprof/Makefile
	etc/Makefile
	examples/Makefile
	examples/AutoTest/Makefile
	examples/Composite/Makefile
	examples/ConfigSample/Makefile
	examples/SimpleIO/Makefile
	examples/SeqIO/Makefile
	examples/SimpleService/Makefile
	examples/ExtTrigger/Makefile
	examples/Templates/Makefile
	examples/tests/Makefile
	docs/Makefile
	win32/Makefile
	win32/OpenRTM-aist/Makefile
	win32/OpenRTM-aist/bin/Makefile
	win32/OpenRTM-aist/etc/Makefile
	win32/OpenRTM-aist/cmake/Makefile
	win32/OpenRTM-aist/docs/Makefile
	win32/OpenRTM-aist/build/Makefile
	win32/OpenRTM-aist/examples/Makefile
	win32/OpenRTM-aist/examples/Composite/Makefile
	win32/OpenRTM-aist/examples/ConfigSample/Makefile
	win32/OpenRTM-aist/examples/SimpleIO/Makefile
	win32/OpenRTM-aist/examples/SeqIO/Makefile
	win32/OpenRTM-aist/examples/SimpleService/Makefile
	win32/OpenRTM-aist/examples/USBCamera/Makefile
	win32/OpenRTM-aist/ext/Makefile
	win32/OpenRTM-aist/ext/sdo/Makefile
	win32/OpenRTM-aist/ext/sdo/observer/Makefile
	win32/OpenRTM-aist/utils/Makefile
	win32/OpenRTM-aist/utils/rtc-template/Makefile
	win32/OpenRTM-aist/utils/rtcd/Makefile
	win32/OpenRTM-aist/utils/rtcprof/Makefile
	win32/OpenRTM-aist/coil/Makefile
	win32/OpenRTM-aist/rtm/Makefile
	win32/OpenRTM-aist/rtm/ext/Makefile
	win32/OpenRTM-aist/rtm/idl/Makefile
	win32/OpenRTM-aist/rtm/idl/device_interfaces/Makefile
	win32/OpenRTM-aist/installer/Makefile
	win32/OpenRTM-aist/installer/Bitmaps/Makefile
	])


