#!/bin/sh

rtc-template -bcxx \
    --module-name=SeqIn --module-type='SequenceInComponent' \
    --module-desc='Sequence InPort component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --inport=Octet:TimedOctet \
    --inport=Short:TimedShort \
    --inport=Long:TimedLong \
    --inport=Float:TimedFloat \
    --inport=Double:TimedDouble \
    --inport=OctetSeq:TimedOctetSeq \
    --inport=ShortSeq:TimedShortSeq \
    --inport=LongSeq:TimedLongSeq \
    --inport=FloatSeq:TimedFloatSeq \
    --inport=DoubleSeq:TimedDoubleSeq \
    --config="data_type:std::string:random"

rtc-template -bcxx \
    --module-name=SeqOut --module-type='SequenceOutComponent' \
    --module-desc='Sequence OutPort component' \
    --module-version=1.0 --module-vendor='Noriaki Ando, AIST' \
    --module-category=example \
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \
    --module-max-inst=10 \
    --outport=Octet:TimedOctet \
    --outport=Short:TimedShort \
    --outport=Long:TimedLong \
    --outport=Float:TimedFloat \
    --outport=Double:TimedDouble \
    --outport=OctetSeq:TimedOctetSeq \
    --outport=ShortSeq:TimedShortSeq \
    --outport=LongSeq:TimedLongSeq \
    --outport=FloatSeq:TimedFloatSeq \
    --outport=DoubleSeq:TimedDoubleSeq
