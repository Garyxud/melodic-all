python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=NXTRTC --module-desc="NXT sample component"^
 --module-version=0.1 --module-vendor=AIST --module-category=example^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --inport=vel:TimedFloatSeq^
 --outport=pos:TimedFloatSeq --outport=sens:TimedFloatSeq^
 --config="map:string:A,B"
