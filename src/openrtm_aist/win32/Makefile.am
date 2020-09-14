## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for win32/OpenRTM-aist
##
## $Id$
##---------------------------------------------------------------------------
SUBDIRS = OpenRTM-aist .


clean-clocal:
	rm -rf ../$(PACKAGE_TARNAME)-$(VERSION)-win32.zip *~


dist-hook:
	rm -rf ../$(PACKAGE_TARNAME)-$(VERSION)-win32.zip
	find ./OpenRTM-aist/ \! -path '*.svn*' \! -path '*CVS*' \
		\! -path '*#*' \! -path '*~' > zip_list
	cat zip_list | zip -@ ../$(PACKAGE_TARNAME)-$(VERSION)-win32.zip
	rm -f zip_list
