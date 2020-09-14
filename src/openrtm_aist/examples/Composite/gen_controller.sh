#!/bin/sh

rtc-template -bcxx \
    --module-name=Controller --module-type='DataFlowComponent' \
    --module-desc='Controller component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --outport=out:TimedFloat \
    --inport=in:TimedFloat


