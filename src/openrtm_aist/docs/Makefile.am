## -*- Makefile -*-
##---------------------------------------------------------------------------
## Makefile.am for swig
##
## $Id$
##---------------------------------------------------------------------------

AUTOMAKE_OPTIONS = 1.4

ClassReference:
	make -f Makefile.docs ClassReference

ClassReference-en:
	make -f Makefile.docs ClassReference-en

IDLReference:
	make -f Makefile.docs IDLReference

IDLReference-en:
	make -f Makefile.docs IDLReference-en

#DevelopersGuide.ps:
#	make -f Makefile.docs DevelopersGuide.ps

#DevelopersGuide.pdf:
#	make -f Makefile.docs DevelopersGuide.pdf

doxygen_classref.conf:
	sed -e 's/__VERSION__/$(VERSION)/g' doxygen_classref.conf.in > doxygen_classref.conf

docdir = $(rtm_docdir)

EXTRA_DIST = \
	doxygen_classref.conf \
	ClassReference \
	ClassReference-en \
	IDLReference \
	IDLReference-en
#	DevelopersGuide.ps \
#	DevelopersGuide.pdf

#install-manual:
#	$(mkinstalldirs) $(DESTDIR)$(pkgdatadir)/docs/
#	if test -f DevelopersGuide.ps; then \
#	  $(INSTALL_DATA) DevelopersGuide.ps $(rtm_docdir); \
#	fi
#	if test -f DevelopersGuide.pdf; then \
#	  $(INSTALL_DATA) DevelopersGuide.pdf $(rtm_docdir); \
#	fi

install-class-reference:
	if test -d ClassReference; then \
	  $(mkinstalldirs) $(DESTDIR)$(rtm_docdir)/ClassReference/html; \
	  for x in ClassReference/html/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(rtm_docdir)/ClassReference/html/;\
	    fi \
	  done \
	fi

install-class-reference-en:
	if test -d ClassReference-en; then \
	  $(mkinstalldirs) $(DESTDIR)$(rtm_docdir)/ClassReference-en/html; \
	  for x in ClassReference-en/html/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(rtm_docdir)/ClassReference-en/html/;\
	    fi \
	  done \
	fi

install-idl-reference:
	if test -d IDLReference; then \
	  $(mkinstalldirs) $(DESTDIR)$(rtm_docdir)/IDLReference/html; \
	  for x in IDLReference/html/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(rtm_docdir)/IDLReference/html/;\
	    fi \
	  done \
	fi

install-idl-reference-en:
	if test -d IDLReference-en; then \
	  $(mkinstalldirs) $(DESTDIR)$(rtm_docdir)/IDLReference-en/html; \
	  for x in IDLReference-en/html/*; do \
	    if test -f $$x; then \
	      $(INSTALL_DATA) $$x $(DESTDIR)$(rtm_docdir)/IDLReference-en/html/;\
	    fi \
	  done \
	fi

#install-data-local: install-class-reference install-class-reference-en install-idl-reference install-idl-reference-en

install-data-local: generate-refs install-refs

generate-refs: ClassReference ClassReference-en IDLReference IDLReference-en

install-refs: install-class-reference install-class-reference-en install-idl-reference install-idl-reference-en

clean-local:
	rm -f Makefile.old *~

maintainer-clean:
	rm -rf \
	Makefile \
	doxygen_classref.conf \
	ClassReference \
	ClassReference-en \
	IDLReference \
	IDLReference-en \
	docs/DevelopersGuide/ClassReference \
	docs/DevelopersGuide/IDLReference
