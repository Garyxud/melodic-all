## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for rtc-template
##
## $Id$
##---------------------------------------------------------------------------

clean-local:
	rm -rf *~ *core *.pyc *.py

src:
	cp -p $(top_srcdir)/utils/rtc-template/rtc-template rtc-template.py
	cp -p $(top_srcdir)/utils/rtc-template/*.py .

wxs: src
	$(top_srcdir)/build/makewxs.py flist \
		-c RtcTemplate \
		-o ../../installer/rtctemplate.yaml \
		-p ..\\utils\\rtc-template \
		*.py

dist-hook: clean-local src wxs
