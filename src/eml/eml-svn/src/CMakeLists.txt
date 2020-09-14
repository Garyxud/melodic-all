FILE( GLOB SRCS al/[^.]*.cxx dll/[^.]*.cxx )
SET (eml_SRCS ${SRCS})

IF (BUILD_FOR_ECOS)
  # Copied some stuff from ECOS_ADD_EXECUTABLE macro, since there is no
  # ECOS_ADD_LIBRARY call
  ADD_DEFINITIONS(-D__ECOS__=1 -D__ECOS=1)
  SET(ECOS_DEFINITIONS -Wall -Wno-long-long -pipe -fno-builtin)

  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-prototypes")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -fno-rtti -Wctor-dtor-privacy -fno-strict-aliasing -fno-exceptions")
  
  # eCos only supports static libraries
  add_library(eml ${eml_SRCS})
  
  # arch specific headers
  FILE( GLOB ARCH_INCLS ${EML_SOURCE_DIR}/include/ethercat/arch-eCos/ethercat/[^.]*.h )
  INSTALL_FILES( /include/ethercat/arch-eCos/ FILES ${ETHERCAT_INCLS})
ENDIF (BUILD_FOR_ECOS)

IF (BUILD_FOR_RTNET)
  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XNPOSIX_USER_CFLAGS}")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  FILE( GLOB RTnet_SRCS arch/RTnet/[^.]*.c)
  SET (eml_SRCS "${eml_SRCS};${RTnet_SRCS}")
  
  # Settings for building a static library (.a)
  OPTION(BUILD_STATIC "Build EML as a static library." OFF)
  IF ( BUILD_STATIC )
    ADD_LIBRARY(eml-static STATIC ${eml_SRCS})
    SET_TARGET_PROPERTIES(eml-static
      PROPERTIES OUTPUT_NAME eml LINK_FLAGS "${XNPOSIX_USER_LDFLAGS}"
      CLEAN_DIRECT_OUTPUT 1)
    INSTALL_TARGETS(/lib eml-static)
  ENDIF ( BUILD_STATIC )

  # Settings for building a dynamic library (.so/.dll)
  OPTION(BUILD_DYNAMIC "Build EML as a dynamic library (Default)." ON)
  IF( BUILD_DYNAMIC)
    ADD_LIBRARY(eml-dynamic SHARED ${eml_SRCS})
    SET_TARGET_PROPERTIES(eml-dynamic
      PROPERTIES OUTPUT_NAME eml LINK_FLAGS "${XNPOSIX_USER_LDFLAGS}"
      CLEAN_DIRECT_OUTPUT 1)
    INSTALL_TARGETS(/lib eml-dynamic)
  ENDIF (BUILD_DYNAMIC)

  # arch specific headers
  FILE( GLOB ARCH_RTNET_INCLS ${EML_SOURCE_DIR}/include/ethercat/arch-RTnet/ethercat/[^.]*.h )
  INSTALL_FILES( /include/ethercat/ FILES ${ARCH_RTNET_INCLS})

ENDIF(BUILD_FOR_RTNET)

IF (BUILD_FOR_POSIX)

  FILE( GLOB posix_SRCS arch/posix/[^.]*.c)
  SET (eml_SRCS "${eml_SRCS};${posix_SRCS}")
  
  # Settings for building a static library (.a)
  OPTION(BUILD_STATIC "Build EML as a static library." OFF)
  IF ( BUILD_STATIC )
    ADD_LIBRARY(eml-static STATIC ${eml_SRCS})
    SET_TARGET_PROPERTIES(eml-static
      PROPERTIES OUTPUT_NAME eml LINK_FLAGS "${POSIX_USER_LDFLAGS}"
      CLEAN_DIRECT_OUTPUT 1)
    INSTALL_TARGETS(/lib eml-static)
  ENDIF ( BUILD_STATIC )

  # Settings for building a dynamic library (.so/.dll)
  OPTION(BUILD_DYNAMIC "Build EML as a dynamic library (Default)." ON)
  IF( BUILD_DYNAMIC)
    ADD_LIBRARY(eml-dynamic SHARED ${eml_SRCS})
    SET_TARGET_PROPERTIES(eml-dynamic
      PROPERTIES OUTPUT_NAME eml LINK_FLAGS "${POSIX_USER_LDFLAGS}"
      CLEAN_DIRECT_OUTPUT 1)
    INSTALL_TARGETS(/lib eml-dynamic)
  ENDIF (BUILD_DYNAMIC)

  # arch specific headers
  FILE( GLOB ARCH_POSIX_INCLS ${EML_SOURCE_DIR}/include/ethercat/arch-RTnet/ethercat/[^.]*.h )
  INSTALL_FILES( /include/ethercat/ FILES ${ARCH_POSIX_INCLS})

ENDIF(BUILD_FOR_POSIX)


# Also install header files
FILE( GLOB AL_INCLS ${EML_SOURCE_DIR}/include/al/[^.]*.h )
INSTALL_FILES( /include/al/ FILES ${AL_INCLS})
FILE( GLOB DLL_INCLS ${EML_SOURCE_DIR}/include/dll/[^.]*.h )
INSTALL_FILES( /include/dll/ FILES ${DLL_INCLS})
#FILE( GLOB ETHERCAT_INCLS ${EML_SOURCE_DIR}/include/ethercat/[^.]*.h )
#INSTALL_FILES( /include/ethercat/ FILES ${ETHERCAT_INCLS})




