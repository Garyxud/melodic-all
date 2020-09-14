## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for rtc-template: rtm template code generator script
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.9

bin_SCRIPTS = rtc-template

yat.py:
	cp -p $(top_srcdir)/build/yat.py .
vcprojtool.py:
	cp -p $(top_srcdir)/build/vcprojtool.py .
slntool.py:
	cp -p $(top_srcdir)/build/slntool.py .

rtc-template: 

pyhelperdir = $(rtm_libdir)/py_helper

pyhelper_SCRIPTS =       \
	yat.py           \
	uuid.py          \
	README_gen.py    \
	profile_gen.py   \
	gen_base.py      \
	cxx_gen.py       \
	cxx_svc_impl.py  \
	vcproj_gen.py    \
	python_gen.py    \
	slntool.py       \
	vcprojtool.py    \
	test-template.py


EXTRA_DIST =             \
	rtc-template     \
	yat.py           \
	uuid.py          \
	README_gen.py    \
	profile_gen.py   \
	gen_base.py      \
	cxx_gen.py       \
	cxx_svc_impl.py  \
	vcproj_gen.py    \
	python_gen.py    \
	slntool.py       \
	vcprojtool.py    \
	test-template.py \
	autotest.sh


clean-local:
	rm -rf *~ *core *.pyc
	rm -rf yat.py slntool.py vcprojtool.py