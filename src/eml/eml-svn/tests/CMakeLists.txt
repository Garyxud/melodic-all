# Make sure the linker can find the eml library once it is built.
LINK_DIRECTORIES (${EML_BINARY_DIR}/src)

IF (BUILD_FOR_ECOS)
    SUBDIRS( ecos )
ENDIF (BUILD_FOR_ECOS)

IF (BUILD_FOR_RTNET)
    SUBDIRS( rtnet )
ENDIF (BUILD_FOR_RTNET)

IF (BUILD_FOR_POSIX)
    SUBDIRS( posix )
ENDIF (BUILD_FOR_POSIX)

