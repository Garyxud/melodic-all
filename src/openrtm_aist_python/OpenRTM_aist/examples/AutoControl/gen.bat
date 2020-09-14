python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=AutoControl --module-desc="Auto controller component for MobileRobot"^
 --module-version=1.0.0 --module-vendor=AIST --module-category=example^
 --module-comp-type=DataFlowComponent --module-act-type=PERIODIC^
 --module-max-inst=1^
 --inport=sens:TimedFloatSeq^
 --outport=vel:TimedFloatSeq
