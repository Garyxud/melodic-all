python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=ConsoleIn --module-desc="Sample component"^
 --module-version=0.1 --module-vendor=S.Kurihara --module-category=Generic^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --outport=Ref:TimedFloat

python %PYTHONPATH%\OpenRTM_aist\utils\rtc-template\rtc-template.py -bpython^
 --module-name=ConsoleOut --module-desc="Sample component"^
 --module-version=0.1 --module-vendor=S.Kurihara --module-category=Generic^
 --module-comp-type=DataFlowComponent --module-act-type=SPORADIC^
 --module-max-inst=10^
 --inport=Ref:TimedFloat
