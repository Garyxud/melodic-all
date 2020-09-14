dnl -*- autoconf -*-
dnl
dnl @file configure.ac
dnl @brief Input file for autoconf
dnl @date $Date$
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
AC_INIT(coil, 1.1.2, n-ando@aist.go.jp, coil)
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([dist-bzip2 tar-pax])
AM_CONFIG_HEADER([include/coil/config_coil.h])
AC_CONFIG_FILES([utils/coil-config/libcoil.pc:utils/coil-config/libcoil.pc.in])

dnl ------------------------------------------------------------
dnl rename config header's PACKAGE_* definition
dnl ------------------------------------------------------------
configdir="include/coil"
mv ${configdir}/config_coil.h.in ${configdir}/config_coil.h.in.org
sed -e 's/undef PACKAGE/undef COIL_PACKAGE/g' ${configdir}/config_coil.h.in.org | sed -e 's/\#undef VERSION.*//g' > ${configdir}/config_coil.h.in
rm -f ${configdir}/config_coil.h.in.org

dnl ------------------------------------------------------------
dnl  Version number
dnl ------------------------------------------------------------
major_version=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\1/g'`]
minor_version=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\2/g'`]
revision_num=[`echo $VERSION | sed -e 's/\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)\.\([0-9]\{1,\}\)/\3/g'`]
short_version=$major_version$minor_version$revision_num

COIL_VERSION=$VERSION
COIL_MAJOR_VERSION=$major_version
COIL_MINOR_VERSION=$minor_version
COIL_REVISION_NUM=$revision_num
COIL_SHORT_VERSION=$short_version
COIL_DESCRIPTION='Common Operating-system Infrastructure Layer'

AC_DEFINE_UNQUOTED([COIL_VERSION],       "$VERSION",       [COIL version])
AC_DEFINE_UNQUOTED([COIL_MAJOR_VERSION], "$major_version", [COIL major version])
AC_DEFINE_UNQUOTED([COIL_MINOR_VERSION], "$minor_version", [COIL minor version])
AC_DEFINE_UNQUOTED([COIL_REVISION_NUM],  "$revision_num",  [COIL revision numver])
AC_DEFINE_UNQUOTED([COIL_SHORT_VERSION], "$short_version", [COIL short version])

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
SHLIB_SUFFIX=".so"

AC_SEARCH_LIBS(pthread_create, pthread c_r, , 
	AC_MSG_ERROR([** thread library is needed **])) 


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
AC_ARG_WITH(ace,              [  --with-ace                      With ACE])
AC_ARG_WITH(ace-includes,     [  --with-ace-includes=dir         Find ACE include files installation below dir(without "include")])
AC_ARG_WITH(ace-lib,          [  --with-ace-lib=dir              Find ACE lib files installation below dir(without "lib"])

dnl  boost directory
AC_ARG_WITH(boost,            [  --with-boost                    With boost])
AC_ARG_WITH(boost-includes,   [  --with-boost-includes=dir       Find boost include files installation below dir])
AC_ARG_WITH(boost-lib,        [  --with-boost-lib=dir            Find boost lib files installation below dir])

dnl Additional compile options
AC_ARG_ENABLE(debug,          [  --enable-debug                  Compile using -g])
AC_ARG_ENABLE(optimize,       [  --disable-optimize              Don't compile using -O])
AC_ARG_WITH(includes,         [  --with-includes=dirs            Additional include dirs (separated by ':')])
AC_ARG_WITH(libs,             [  --with-libs=dirs                Additional lib dirs (separated by ':')])
AC_ARG_ENABLE(gcc2option,     [  --with-gcc2                     Compile with gcc2])
AC_ARG_ENABLE(artlinux,       [  --with-artlinux                 With ART-Linux])
AC_ARG_ENABLE(artlinux,       [  --with-rdtsc                    With Read Time Stamp Counter])
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
	AC_DEFINE([COIL_OS_FREEBSD], [TRUE], [OS is FreeBSD])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[Linux*)]
	AC_DEFINE([COIL_OS_LINUX], [TRUE], [OS is Linux])
	AC_DEFINE([_REENTRANT], [TRUE], [Multi Thread Support])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	LDFLAGS="$LDFLAGS -luuid -ldl"
	COIL_PLATFORM=posix
	;;	
[HP-UX*)]
	AC_DEFINE([COIL_OS_HPUX], [TRUE], [OS is HP-UX])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[SunOS*)]
	AC_DEFINE([COIL_OS_SUNOS], [TRUE], [OS is SunOS])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[IRIX*)]
	AC_DEFINE([COIL_OS_IRIX], [TRUE], [OS is IRIX])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[OSF1*)]
	AC_DEFINE([COIL_OS_OSF1], [TRUE], [OS is OSF1])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[Darwin*)]
	AC_DEFINE([COIL_OS_DARWIN], [TRUE], [OS is Max OS X])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[QNX*)]
	AC_DEFINE([COIL_OS_QNX], [TRUE], [OS is QNX])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	;;	
[CYGWIN*)]
	AC_DEFINE([COIL_OS_CYGWIN], [TRUE], [OS is Cygwin])
	AC_DEFINE([COIL_PLATFORM], [POSIX], [Platform is POSIX])
	COIL_PLATFORM=posix
	CPPFLAGS="-DCygwin $CPPFLAGS"
	CXXFLAGS="-DCygwin $CXXFLAGS"
	LDFLAGS="-no-undefined $LDFLAGS -luuid -ldl"
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


dnl ============================================================
dnl Check ACE setup
dnl
dnl 1. check ace/ACE.h for ACE
dnl 2. check libACE.so
dnl 3. check ACE version from ace/Version.h
dnl ============================================================
dnl ------------------------------------------------------------
dnl  Find ace/ACE.h
dnl ------------------------------------------------------------
echo "have_ace: " $have_ace
echo "with_ace: " $with_ace

have_ace=no
if test "x$with_ace" != "x" ; then
  AC_MSG_CHECKING(for ace/ACE.h)
  dnl PKG_CHECK_MODULES(ACE,ACE)
  FP_FIND_FILE(ace/ACE.h,
    $with_ace_includes $more_incs $base_incs $base_dirs,
    ACE_wrappers include cxx/include)
  if test "x$ace_ACE_h_dir" = "x" ; then
     AC_MSG_RESULT(ACE (ace/ACE.h) not found)
     AC_MSG_ERROR(
	use --with-ace-includes=<dir> to locate ace/ACE.h
	hint: <dir>/( ACE_wrappers | include | cxx/include ) will be searched.)
	# Error exit!!
  else
    AC_MSG_RESULT(found in $ace_ACE_h_dir)
    have_ace=yes
  fi
fi

dnl ------------------------------------------------------------
dnl  Find libACE.so / Checking ACE version
dnl ------------------------------------------------------------
if test "x$have_ace" = "xyes" ; then
	AC_MSG_CHECKING(for libACE.so)
	FP_FIND_FILE(libACE.so,
		$with_ace_lib $more_libs $base_libs $base_dirs,
		ACE_wrappers/ace/lib lib cxx/lib)
	if test "x$libACE_so_dir" = "x" ; then
		AC_MSG_RESULT(not found)
		AC_MSG_ERROR(
	use --with-ace-lib=<dir> to locate libACE.so.
hint: <dir>/( ACE_wrappers/ace/lib lib cxx/lib ) will be searched.)
		# Error exit!!
	else
		AC_MSG_RESULT(found in $libACE_so_dir)
	fi

	AC_MSG_CHECKING(ACE version)

	# In ACE 5.5.4, ACE_VERSION is defined as ACE_VERSION " 5.5.4"
	# "cut" cannot be used to extract version strings
	ace_version=[`grep ACE_VERSION $ace_ACE_h_dir/ace/Version.h | sed 's/.*\"\(.*\)\"/\1/' | sed 's/ //g'`]

	case $ace_version in
	[5.6.1|5.6|5.5.*|5.4.*|5.3.*|5.5|5.4|5.3)]
		AC_DEFINE([ACE_5_6_1_OR_EARLIER], [TRUE], [ACE version 5.6.1 or earlier])
		;;
	# ACE 5.6.2 or later signature of UUID_Generator::generateUUID
	# has been changed as generate_UUID
	[5.7.*|5.6.[2-9]|5.7|5.6)]
		;;
	*)
		AC_MSG_WARN([Unknown ACE version number, expecting 5.3.0 or above.])
	esac
 	AC_MSG_RESULT($ace_version)

	#
	# ACE setup complete
	#
	# Compiler/Linker options
	CPPFLAGS="-I$ace_ACE_h_dir $CPPFLAGS"
	CXXFLAGS="-I$ace_ACE_h_dir $CXXFLAGS"
	LDFLAGS="-L$libACE_so_dir $LDFLAGS "
	LIBS="-lACE $LIBS"
	LDSOLIBS="-lACE $LDSOLIBS"
	AC_DEFINE([WITH_ACE], [TRUE], [compiled with ACE])
	AC_DEFINE([COIL_PLATFORM], [ACE], [coil platform is ACE])
	COIL_PLATFORM=ace
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

dnl ------------------------------------------------------------
dnl ARTLinux setup
dnl
dnl ------------------------------------------------------------
have_artlinux=no

if test "x$with_artlinux" != "x" ; then
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
	fi
	AC_DEFINE([RTM_OS_ARTLINUX], [TRUE], [ARTLinux is enable])
	ARTLIB="$art_syscalls_o_dir/art_syscalls.o"
	echo $ARTLIB
	echo 'RTM is compiled with ARTLinux'
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
AC_SUBST(LD)
AC_SUBST(LDFLAGS)
AC_SUBST(LIBS)
AC_SUBST(LDSO)
AC_SUBST(SOEXT)
AC_SUBST(LDSOOPTS)
AC_SUBST(LDSOLIBS)
AC_SUBST(pkgconfigdir)

dnl RTM version related options
AC_SUBST(COIL_VERSION)
AC_SUBST(COIL_MAJOR_VERSION)
AC_SUBST(COIL_MINOR_VERSION)
AC_SUBST(COIL_REVISION_NUM)
AC_SUBST(COIL_SHORT_VERSION)
AC_SUBST(COIL_PLATFORM)
AC_SUBST(COIL_DESCRIPTION)
dnl ORB related options
AC_SUBST(ORB)
AC_SUBST(IDLC)
AC_SUBST(IDL_FLAGS)
AC_SUBST(COSNAMES)

dnl Wrapper code subdirectory 
AC_SUBST(WRAPPERS)
AC_SUBST(PYTHON_FLAGS)
dnl ARTLinux lib
AC_SUBST(ARTLIB)
AC_SUBST(UNITTEST)

AM_CONDITIONAL(COIL_PLATFORM_ACE, test "x$have_ace" = "xyes")
AM_CONDITIONAL(COIL_PLATFORM_POSIX, test "x$have_ace" != "xyes")

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

dnl OpenRTM's includedir/libdir/datadir definition
coil_includedir=$includedir"/coil-"$major_version"."$minor_version
coil_libdir=$libdir"/coil-"$major_version"."$minor_version
coil_datadir=$datadir"/coil-"$major_version"."$minor_version

AC_SUBST(coil_libdir)
AC_SUBST(coil_datadir)
AC_SUBST(coil_includedir)

echo ''
echo '------------------------------------------------------------'
echo ''
echo ' '$PACKAGE_NAME' will be build with the following options. '
echo ''
echo '------------------------------------------------------------'
echo 'coil platform: ' $COIL_PLATFORM
echo 'coil system: ' $system
echo ''
echo 'CXX: '$CXX
echo 'CPPFLAGS: '$CPPFLAGS
echo 'CXXFLAGS: '$CXXFLAGS
echo ''
echo 'LD: '$LD
echo 'LIBS: '$LIBS
echo 'LDFLAGS: '$LDFLAGS
echo 'LDSOLIBS: '$LDSOLIBS
echo '------------------------------------------------------------'
echo ''


AC_OUTPUT([Makefile
	build/Makefile
	common/Makefile
	include/Makefile
	include/coil/Makefile
	lib/Makefile
	posix/Makefile
	posix/coil/Makefile
	win32/Makefile
	win32/bin/Makefile
	win32/build/Makefile
	win32/coil/Makefile
	ace/Makefile
	ace/bin/Makefile
	ace/build/Makefile
	ace/coil/Makefile
	tests/Makefile
	tests/stringutil/Makefile
	tests/Properties/Makefile
	tests/Condition/Makefile
	tests/DynamicLib/Makefile
	tests/Factory/Makefile
	tests/File/Makefile
	tests/Guard/Makefile
	tests/Listener/Makefile
	tests/Mutex/Makefile
	tests/OS/Makefile
	tests/Signal/Makefile
	tests/Singleton/Makefile
	tests/Task/Makefile
	tests/Time/Makefile
	tests/Timer/Makefile
	tests/TimeValue/Makefile
	tests/UUID/Makefile
	tests/TimeMeasure/Makefile
	tests/AsyncInvoker/Makefile
	tests/Logger/Makefile
	tests/PeriodicTask/Makefile
	utils/Makefile
	utils/coil-config/Makefile
	])



