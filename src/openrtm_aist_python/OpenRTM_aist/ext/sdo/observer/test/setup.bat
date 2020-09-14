@echo off
echo "<<< ComponentObserverConsumer Test setup start >>>"

set idlfiles=BasicDataType.idl DataPort.idl OpenRTM.idl RTC.idl SDOPackage.idl

rem # idl file copy
for %%x in (%idlfiles%) do copy ..\..\..\..\RTM_IDL\%%x .

copy ..\ComponentObserver.idl .

rem # idl file compile
set idlfiles=%idlfiles% ComponentObserver.idl
for %%x in (%idlfiles%) do omniidl -I. -bpython %%x

echo "<<< ComponentObserverConsumer Test setup Complete >>>"
echo ""

