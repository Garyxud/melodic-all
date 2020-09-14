#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT(doil-servant-unittest, 1.0, ADDRESS)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_HEADER([config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for libraries.
# FIXME: Replace `main' with a function in `-lcppunit':
AC_CHECK_LIB([cppunit], [main])
AC_PROG_RANLIB

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL

# Checks for library functions.

AC_CONFIG_FILES([Makefile
                 ConfigurationServant/Makefile
                 ComponentActionServant/Makefile
                 DataFlowComponentActionServant/Makefile
                 DataFlowComponentServant/Makefile
                 ExecutionContextServant/Makefile
                 ExecutionContextServiceServant/Makefile
                 FsmObjectServant/Makefile
                 FsmParticipantActionServant/Makefile
                 FsmParticipantServant/Makefile
                 FsmServant/Makefile
                 FsmServiceServant/Makefile
                 LightweightRTObjectServant/Makefile
                 ModeCapableServant/Makefile
                 ModeServant/Makefile
                 MonitoringServant/Makefile
                 MultiModeComponentActionServant/Makefile
                 MultiModeObjectServant/Makefile
                 OrganizationServant/Makefile
                 PortServiceServant/Makefile
                 RTObjectServant/Makefile
                 SDOServant/Makefile
                 SDOServiceServant/Makefile
                 SDOSystemElementServant/Makefile
                 stubs/Makefile
                 ])
AC_OUTPUT
