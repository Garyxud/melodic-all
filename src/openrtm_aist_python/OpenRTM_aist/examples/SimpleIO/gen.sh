#!/bin/sh

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=ConsoleIn --module-type='DataFlowComponent' \
    --module-desc='Console input component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 --outport=out:TimedLong

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=ConsoleOut --module-type='DataFlowComponent' \
    --module-desc='Console output component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 --inport=in:TimedLong


