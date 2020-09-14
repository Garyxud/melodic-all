#!/bin/sh

rtc-template -bcxx \
    --module-name=Motor --module-type='DataFlowComponent' \
    --module-desc='Motor component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --outport=out:TimedLong \
    --inport=in:TimedFloat \
    --config=motor_id:int:0


