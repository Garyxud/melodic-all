#!/bin/sh

python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=Controller --module-type='DataFlowComponent'^
 --module-desc='Controller component'^
 --module-version=1.0 --module-vendor='Noriaki Ando, AIST'^
 --module-category=example^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --outport=out:TimedFloat^
 --inport=in:TimedFloat


python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=Motor --module-type='DataFlowComponent'^
 --module-desc='Motor component'^
 --module-version=1.0 --module-vendor='Noriaki Ando, AIST'^
 --module-category=example^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --outport=out:TimedLong^
 --inport=in:TimedFloat^
 --config=motor_id:int:0

python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=Sensor --module-type='DataFlowComponent'^
 --module-desc='Sensor component'^
 --module-version=1.0 --module-vendor='Noriaki Ando, AIST'^
 --module-category=example^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --outport=out:TimedFloat^
 --inport=in:TimedLong
