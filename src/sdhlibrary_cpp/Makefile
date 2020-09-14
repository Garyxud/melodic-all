#=======================================================================
#//! \file
#//! \section sdhlibrary_cpp_makefile_general General file information
#//!   \author    Dirk Osswald 
#//!   \date      2007-01-03
#//!  
#//! \brief  
#//!   Makefile for SDH SDHLibrary C project.
#//!
#//!   This makefile can generate the C library itself, demo-programs and auxiliary stuff like
#//!   doxygen documentation or generate a distribution for delivery to end users.
#//!   
#//!   For a general description of the project see \ref sdhlibrary_cpp_dox_general "general project information".
#//!
#//! \section sdhlibrary_cpp_makefile_variables Makefile variables
#//!   The variables defined here state project specific settings which are
#//!   then used by the goals and/or by the included, more generic sub makefiles
#//!   like:
#//!   - \ref Makefile-common "Makefile-common"
#//!   - \ref Makefile-doc    "Makefile-doc"
#//!   - \ref Makefile-rules  "Makefile-rules"
#//! 
#//! \section sdhlibrary_cpp_makefile_targets Makefile targets
#//!   - \b \c all : generate everything
#//!     - \b \c build  : generate library and demo programs
#//!     - \b \c doc    : generate all documentation
#//!   - \b \c clean    : clean up generated program files, but not TAGS or doxygen doc
#//!   - \b \c mrproper : clean up all generated files, including TAGS and doxygen doc
#//!   - \b \c tags     : generate emacs TAGS file
#//!
#//! \section sdhlibrary_cpp_makefile_links Links
#//!   - The online documentation for \c gnu \c make can be found at
#//!     <a href="http://www.gnu.org/software/make/manual/make.html">
#//!     http://www.gnu.org/software/make/manual/make.html</a>
#//!  
#//! \section sdhlibrary_cpp_makefile_copyright Copyright
#//!
#//!  Copyright (c) 2007 SCHUNK GmbH & Co. KG
#//!
#//!  <HR>
#//!  \internal
#//!
#//!    \subsection sdhlibrary_cpp_makefile_details SVN related, detailed file specific information:
#//!      $LastChangedBy: Osswald2 $
#//!      $LastChangedDate: 2010-04-26 15:06:01 +0200 (Mo, 26 Apr 2010) $
#//!      \par SVN file revision:
#//!        $Id: Makefile 5547 2010-04-26 13:06:01Z Osswald2 $
#//!
#//!  \subsection sdhlibrary_cpp_makefile_changelog Changelog of this file:
#//!      \include Makefile.log
#//!
#=======================================================================
#//! \cond ignore_me   doxygen cannot parse Makefiles, so just ignore it

.DEFAULT_GOAL := all


BASEDIR = .

include ${BASEDIR}/Makefile-settings

## specify target specific definitions for TAGFILES:
#  (links to internal tag files make sense only for the internal docu,
#   for the external docu it would lead to broken links)
doc_internal_html: TAGFILES=../../../../tags/Protocol.tag=../../../../../../Protocol/doc/internal/html/ ../../../../tags/MotionControl.tag=../../../../../../MotionControl/doc/internal/html/ ../../../../tags/common.tag=../../../../../../common/doc/internal/html/
doc_internal: DOC_EXTRA_INPUT=
doc_internal: DOC_EXTRA_EXCLUDE=
doc_internal: DOC_EXTRA_EXCLUDE_PATTERNS=

doc_external_html: TAGFILES=
doc_external: DOC_EXTRA_EXCLUDE=demo/demo-ref.cpp demo/demo-test.cpp
doc_external: DOC_EXTRA_EXCLUDE_PATTERNS=

#
########################################################################



########################################################################
# now the goals

# list of sub directories with code (with the new Makefile.subdir this now does work on Linux)
SUBDIRS = sdh demo 


# Default goal: Make all (programs and documentation)
.PHONY: all
all: build 


# Build the library
.PHONY: build
build: TARGET=build
# cygwin gcc-4.3 is missing a newline after some messages which
# prevent eclipse from locating following messages. So use
# an output filter to correct this.
#build: export MAKE_OUTPUT_FILTER = 2>&1 | sed --unbuffered "s/This should work unless it involves constant data structures referencing symbols from auto-imported DLLs./This should work unless it involves constant data structures referencing symbols from auto-imported DLLs.\\\n/"
build: recursive

# Build the library only
.PHONY: build_lib
build_lib: TARGET=build_lib
build_lib: recursive

# Build the demo programs only
.PHONY: build_demo
build_demo: TARGET=build_demo
build_demo: recursive

# Build the test programs only
.PHONY: build_test
build_test: TARGET=build_test
build_test: recursive

tstinfo: TARGET=tstinfo
tstinfo: recursive

# Create and install generated files 
.PHONY: install
# let sub-makefiles install their part first:
install: TARGET=install
install: recursive 
#	# then install docu:
	@echo "installing documentation in ${INSTALL_DIR_DOC}"
	@if [ "${INSTALL_DIR_DOC}" != "" ]; then \
	  install -d ${INSTALL_DIR_DOC} ; \
	  cp -a ${DOCDIR}/*.html ${DOCDIR}/${SDHLIBRARY_NAME}-*.pdf ${DOCDIR}/external/ \
	    ${INSTALL_DIR_DOC} ; \
	  if [ -e "${DOCDIR}/internal/" ]; then \
	    cp -a ${DOCDIR}/internal/ \
	      ${INSTALL_DIR_DOC} ; \
	  fi ; \
	fi  

# uninstall previously installed files 
.PHONY: uninstall
# let sub-makefiles uninstall their part first:
uninstall: TARGET=uninstall
uninstall: recursive
#	# then uninstall docu:
	@echo "uninstalling documentation from ${INSTALL_DIR_DOC}"
	@if [ "${INSTALL_DIR_DOC}" != "" ]; then \
	  rm -rf ${INSTALL_DIR_DOC} ; \
	fi  


# Clean up generated files but not doc or tags.
.PHONY: clean
clean: TARGET=clean
clean: recursive


# Clean up generated files including doc and tags.
.PHONY: mrproper
mrproper: TARGET=mrproper
mrproper: clean doc_clean
mrproper: recursive

.PHONY: tidy
tidy: doc/index-overview.tidy doc/troubleshooting.tidy

######################################################################
#

# some common settings and targets are defined separately to keep this
# main Makefile more concise
include ${COMMON_DIR}/Makefile-common

# The variables and targets to generate documentation are defined
# separately to keep this main Makefile clearly laid out.
include ${COMMON_DIR}/Makefile-doc

# The variables and targets to call make recursively are defined
# separately to keep this main Makefile clearly laid out.
include ${COMMON_DIR}/Makefile-subdir

## For internal use only:
#  The variables and targets to generate a distribution are defined
#  separately to keep this main Makefile clearly laid out.
-include Makefile-dist

#
########################################################################

########################################################################
# additional rules for including the online help of the demo programs
# into the doxygen documentation

## the programs to extract the online help from
ONLINE_HELP_EXES = ${patsubst %.stackdump,,${wildcard ${BINDIR}/demo-*}}

## the dox files to generate
ONLINE_HELP_DOX = ${patsubst ${BINDIR}/demo-%,${DOCDIR}/onlinehelp-demo-%.dox,${ONLINE_HELP_EXES}}

## add dependencies to the doc_*_html targets
doc_external_html: ${ONLINE_HELP_DOX} 
doc_internal_html: ${ONLINE_HELP_DOX} 

## add extra dependency to the doc_clean target
doc_clean: doc_clean_onlinehelp

## pattern rule for generating the onlinehelp-*.dox file from the program
${DOCDIR}/onlinehelp-%.dox: ${BINDIR}/%
	@echo "Generating $@ from $<:"
	@echo -e "// auto generated file, do not edit!\n/*!\n  \\\file\n\n  \\\addtogroup sdh_library_cpp_onlinehelp_group\n  @{\n\n  \\\par Online help for program \\\c $*\n\n \\\code\n" > $@
	$< --help               >> $@
	@echo -e "\n\n\\\endcode\n  @} \n*/"   >> $@
	@echo -e "finished $@\n"

## rule to clean the generated onlinehelp-*.dox files
doc_clean_onlinehelp:
	rm -f ${ONLINE_HELP_DOX}
#
########################################################################


#-----------------------------------------------------------------------
# emacs settings:
# Local Variables:
# mode: Makefile
# End:
#-----------------------------------------------------------------------
#//! \endcond
########################################################################
