all: RTC
OMNIIDL=PYTHONPATH=$(PYTHONPATH):../../ omniidl

CXXFLAGS= \
	-fPIC -c \
	-I. \
	-I../../../../ \
	-I../../../../coil/include \
	-I../../../../rtc/corba/idl

SERVANTS_OBJ=\
	ComponentActionServant.o \
	ConfigurationServant.o \
	DataFlowComponentActionServant.o \
	DataFlowComponentServant.o \
	ExecutionContextServant.o \
	ExecutionContextServiceServant.o \
	FsmObjectServant.o \
	FsmParticipantActionServant.o \
	FsmParticipantServant.o \
	FsmServant.o \
	FsmServiceServant.o \
	LightweightRTObjectServant.o \
	ModeCapableServant.o \
	ModeServant.o \
	MonitoringServant.o \
	MultiModeComponentActionServant.o \
	MultiModeObjectServant.o \
	OrganizationServant.o \
	PortServiceServant.o \
	RTObjectServant.o \
	SDOServant.o \
	SDOServiceServant.o \
	SDOSystemElementServant.o
%.o : %.c
	$(CC) -c $(CXXFLAGS) $< -o $@

help:
	$(OMNIIDL) -bdoil -u

#		-Wbm=rtc_doil.map 

interface:
	rm -f *.gch
	$(OMNIIDL) -bdoil -Wbi=SDOPackageTypes.h \
		-Wbinterface -Wbimplicit \
		-Wbins=Local ./RTC.idl
	$(OMNIIDL) -bdoil \
		-Wbinterface -Wbimplicit \
		-Wbins=Local ./SDOPackage.idl
#	g++ -I../../../../ -I. I*.h *Types.h

servant:
	rm -f *.gch *Servant.h
	$(OMNIIDL) -bdoil -Wbi=SDOPackageTypes.h \
		-Wbi=rtc/corba/idl/RTCSkel.h \
		-Wbi=SDOPackageTypeConversion.h \
		-Wbcorbaservant -Wbimplicit \
		-Wbins=Local -Wbsns=CORBA ./RTC.idl
	$(OMNIIDL) -bdoil -Wbi=rtc/corba/idl/SDOPackageSkel.h \
		-Wbcorbaservant -Wbimplicit \
		-Wbins=Local -Wbsns=CORBA ./SDOPackage.idl
	g++ $(CXXFLAGS)  SDOPackageTypeConversion.cpp
	g++ -shared -o SDOPackageTypeConversion.so SDOPackageTypeConversion.o
#	strip SDOPackageTypeConversion.so
	g++ $(CXXFLAGS) RTCTypeConversion.cpp
	g++ -shared -o RTCTypeConversion.so RTCTypeConversion.o
#	strip RTCTypeConversion.so
	g++ -shared -o TypeConversion.so SDOPackageTypeConversion.o RTCTypeConversion.o
#	strip TypeConversion.so
	ls -al *.so
#	g++ $(CXXFLAGS) *Servant.h

servant_o: $(SERVANTS_OBJ)

adapter:
	rm -f *.gch
	$(OMNIIDL) -bdoil -Wbi=SDOPackageTypes.h \
		-Wbcorbaadapter -Wbimplicit \
		-Wbans=CORBA ./RTC.idl
	$(OMNIIDL) -bdoil \
		-Wbcorbaservant -Wbimplicit \
		-Wbans=CORBA ./SDOPackage.idl
	g++ -I../../../../ -I../../../../coil/include *.h


clean:
	rm -f *~
	rm -f *Servant*
	rm -f *Adapter*
	rm -f *TypeConversion*
	rm -f *Types*
	rm -f I*.h
	rm -f *.gch
