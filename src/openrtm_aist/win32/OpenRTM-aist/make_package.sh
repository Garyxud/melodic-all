#!/bin/sh
#
# @brief Archiving OpenRTM-aist Windows binaries in to a ZIP file
# @author Noriaki Ando <n-ando@aist.go.jp>
#         Copyright 2014 (C) All Right Reserved
#
# This is part of OpenRTM-aist build scripts suite
# for building OpenRTM-aist for Windows binaries.
#
# The following environment variables should be set.
# ex.
if test "x$OPENRTM_DIR" = "x" ; then
    export OPENRTM_DIR=OpenRTM-aist
fi
if test "x$PYTHON_DIR" = "x" ; then
    export PYTHON_DIR=/cygdrive/c/Python27
fi
if test "x$VC_VERSION" = "x" ; then
    export VC_VERSION=10
fi
if test "x$ARCH" = "x" ; then
    export ARCH=x86
fi
export PATH=${PATH}:/bin:/usr/bin
#
#==============================
# main
if test "x$OPENRTM_DIR" = "x" ; then
	echo "OPENRTM_DIR not defined"
	exit -1
fi

/usr/bin/find ./$OPENRTM_DIR -type d -exec chmod 755 {} \;
/usr/bin/find ./$OPENRTM_DIR -type f -exec chmod 644 {} \;


echo "Current dir: " `pwd`
echo "OPENRTM_DIR: " ${OPENRTM_DIR}
echo "Removing auto generated temp files."
/usr/bin/find ./$OPENRTM_DIR -name '*.pyc' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name '*.sdf' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name '*.suo' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name '*.yaml' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name 'Makefile' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name 'Makefile.in' -type f -exec rm {} \;
/usr/bin/find ./$OPENRTM_DIR -name 'Makefile.am' -type f -exec rm {} \;

dirs="rtm examples utils"
for d in $dirs ; do
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.obj' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.pdb' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.tlog' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.dll' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.lib' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.exp' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.def' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.res' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.rc' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.exe' -type f -exec rm {} \;
	/usr/bin/find ./$OPENRTM_DIR/$d/ -name '*.user' -type f -exec rm {} \;
done

echo "done"

if test "x$ARCH" = "xx86" ; then
	WIN_ARCH=win32
elif test "x$ARCH" = "xx86_64" ; then
	WIN_ARCH=win64
else
	echo "Unknown architecture: " $ARCH
	exit -1
fi

if test "x$PYTHON_DIR" = "x" ; then
	echo "PYTHON_DIR is not defined. "
	exit -1
else
	PY_VER=`echo $PYTHON_DIR | sed 's/.*[Pp][Yy][Tt][Hh][Oo][Nn]\([0-9][0-9]\).*/\1/'`
fi

OPENRTM_VER=`grep '^name' ${OPENRTM_DIR}/rtm/version.txt | awk '{print $3;}'`

NEW_DIR=${OPENRTM_VER}-${WIN_ARCH}-vc${VC_VERSION}
ZIP_FILE=${OPENRTM_VER}-${WIN_ARCH}-vc${VC_VERSION}.zip

echo "NEW_DIR: " ${NEW_DIR}
echo "OPENRTM_DIR: " ${OPENRTM_DIR}
echo "ZIP_FILE: " ${ZIP_FILE}

rm -rf ${NEW_DIR}
cp -r ${OPENRTM_DIR} ${NEW_DIR}

rm -rf ${ZIP_FILE}
zip -r ${ZIP_FILE} ${NEW_DIR}
rm -rf ${NEW_DIR}

# end of script
#==============================