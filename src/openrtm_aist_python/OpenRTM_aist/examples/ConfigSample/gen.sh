#!/bin/sh

python ${PYTHONPATH}/OpenRTM_aist/utils/rtc-template/rtc-template.py -bpython \
    --module-name=ConfigSample --module-type='DataFlowComponent' \
    --module-desc='Configuration example component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --config=int_param0:int:0 --config=int_param1:int:1 \
    --config=double_param0:double:0.11 --config=double_param1:double:9.9 \
    --config="str_param0:std::string:hoge" \
    --config="str_param1:std::string:dara" \
    --config="vector_param0:float:0.0,1.0,2.0,3.0,4.0"

