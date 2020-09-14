## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for coil (Component Object Infrastructure Layer)
##
## $Id$
##---------------------------------------------------------------------------

SUBDIRS =            \
	stringutil   \
	Properties   \
        Condition    \
        DynamicLib   \
        Factory      \
        File         \
        Guard        \
        Listener     \
        Logger       \
	Mutex        \
        OS           \
        PeriodicTask \
        Signal       \
        Singleton    \
	Task         \
        Time         \
	Timer        \
	TimeValue    \
	UUID         \
	TimeMeasure  \
	AsyncInvoker




clean-local:
	rm -f *~
	rm -f *.sln *.ncb *.suo

EXTRA_DIST =                    \
	coil_test.vcproj.yaml   \
	coil_test.sln.dep       \
	coiltest_config.vsprops \
	coiltest_vc8.sln        \
	coiltest_vc9.sln
#	coiltest_vc10.sln

#------------------------------------------------------------
# sln file build rules
#------------------------------------------------------------
win32_builddir = .

#sln: coiltest_vc8.sln coiltest_vc9.sln coiltest_vc10.sln
sln: coiltest_vc8.sln coiltest_vc9.sln

vcproj:
	(for testdir in $(SUBDIRS) ; do cd $$testdir ; make vcproj ; cd .. ; done)


coiltest_vc8.sln: coil_test.sln.dep vcproj
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC8            \
		--dep coil_test.sln.dep \
		--out coiltest_vc8.sln \
		`find ./ -name '*_vc8.vcproj'`
	qkc -O- -ms coiltest_vc8.sln

coiltest_vc9.sln: coil_test.sln.dep vcproj
	$(top_srcdir)/build/slntool.py     \
		--vcversion VC9            \
		--dep coil_test.sln.dep \
		--out coiltest_vc9.sln \
		`find ./ -name '*_vc9.vcproj'`
	qkc -O- -ms coiltest_vc9.sln

#coiltest_vc10.sln: coil_test.sln.dep vcproj
#	$(top_srcdir)/build/slntool.py     \
		--vcversion VC10            \
		--dep coil_test.sln.dep \
		--out coiltest_vc10.sln \
		`find ./ -name '*_vc10.vcproj'`
#	qkc -O- -ms coiltest_vc10.sln


