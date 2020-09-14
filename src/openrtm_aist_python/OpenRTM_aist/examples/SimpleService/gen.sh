#!/bin/sh

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=MyServiceProvider --module-type="MyServiceProvider" \
    --module-desc="MyService Provider Sample component" \
    --module-version=0.1 --module-vendor=AIST --module-category=Generic \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --service=MyService:myservice0:MyService \
    --service-idl=MyService.idl

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=MyServiceConsumer --module-type="MyServiceConsumer" \
    --module-desc="MyService Consumer Sample component" \
    --module-version=0.1 --module-vendor=AIST --module-category=Generic \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --consumer=MyService:myservice0:MyService \
    --consumer-idl=MyService.idl
