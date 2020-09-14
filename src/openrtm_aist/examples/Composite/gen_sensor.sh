#!/bin/sh

rtc-template -bcxx \
    --module-name=Sensor --module-type='DataFlowComponent' \
    --module-desc='Sensor component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --outport=out:TimedFloat \
    --inport=in:TimedLong
