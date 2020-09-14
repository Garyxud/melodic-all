#!/bin/sh

# メモリーリークチェックの環境セットアップ
echo "<<< Memory leak check setup start >>>"

# utils/rtm-naming copy
cp -pf ../../utils/rtm-naming/rtm-naming .

# utils/rtcd copy
cp -af ../../utils/rtcd/.deps/ .
cp -af ../../utils/rtcd/.libs/ .
cp -pf ../../utils/rtcd/rtcd .

# examples/*.so file copy
cp -pf ../Composite/.libs/*.so .
cp -pf ../ConfigSample/.libs/*.so .
cp -pf ../SeqIO/.libs/*.so .
cp -pf ../SimpleIO/.libs/*.so .
cp -pf ../SimpleService/.libs/*.so .

# idl file copy
cp -pf ../../src/lib/rtm/idl/BasicDataType.idl .
cp -pf ../../src/lib/rtm/idl/Manager.idl .
cp -pf ../../src/lib/rtm/idl/RTC.idl .
cp -pf ../../src/lib/rtm/idl/SDOPackage.idl .
cp -pf ../../src/lib/rtm/idl/DataPort.idl .
cp -pf ../../src/lib/rtm/idl/OpenRTM.idl .

# idl file compile
omniidl -bpython *.idl

echo "<<< Memory leak check setup Complete >>>"
echo ""

