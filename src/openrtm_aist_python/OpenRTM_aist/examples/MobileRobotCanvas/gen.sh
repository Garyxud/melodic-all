#!/bin/sh

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=MobileRobotCanvas --module-type='DataFlowComponent' \
    --module-desc='sample component for Python and Tkinter' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 --inport=vel:TimedFloatSeq

