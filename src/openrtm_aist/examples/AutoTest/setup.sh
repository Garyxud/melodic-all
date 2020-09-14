#!/bin/sh

# コンポーネント接続テスト
echo "<<< Connect Test setup start >>>"

# idl file copy
cp -pf ../../src/lib/rtm/idl/BasicDataType.idl .
cp -pf ../../src/lib/rtm/idl/Manager.idl .
cp -pf ../../src/lib/rtm/idl/RTC.idl .
cp -pf ../../src/lib/rtm/idl/SDOPackage.idl .
cp -pf ../../src/lib/rtm/idl/DataPort.idl .
cp -pf ../../src/lib/rtm/idl/OpenRTM.idl .

# idl file compile
omniidl -bpython *.idl

echo "<<< Connect Test setup Complete >>>"
echo ""

