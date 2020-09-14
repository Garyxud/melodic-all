#!/bin/sh

testdir=gentest
build_log=build.log

#------------------------------------------------------------
# copying *.py into py_helper
if test \! -d py_helper; then
    mkdir py_helper
    cp *.py py_helper
fi

#------------------------------------------------------------
# create build dir
rm -rf $testdir
mkdir $testdir
cd $testdir

#------------------------------------------------------------
# generate project
if test \! -f ../test-template.py; then
    echo "test-template.py not found. Abort"
    exit -1
fi
python ../test-template.py
chmod 755 build.sh
sh build.sh



