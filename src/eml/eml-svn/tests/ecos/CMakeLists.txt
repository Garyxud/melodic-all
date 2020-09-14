# INCLUDE(ProjectSources.txt)

SET(CMAKE_VERBOSE_MAKEFILE ON)
#create the eCos executable
ADD_DEFINITIONS(-D__ECOS__=1 -D__ECOS=1)
SET(ECOS_DEFINITIONS -Wall -Wno-long-long -pipe -fno-builtin)

SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-prototypes")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -fno-rtti -Wctor-dtor-privacy -fno-strict-aliasing -fno-exceptions")

#sources depend on target.ld
FOREACH (_current_FILE ${ARGN})
ADD_FILE_DEPENDENCIES(${_current_FILE} ${ECOS_INSTALL_PATH}/ecos/install/lib/target.ld)
ENDFOREACH (_current_FILE)

#use a variable for the make_clean_files since later on even more files are added
SET(ECOS_ADD_MAKE_CLEAN_FILES ${ECOS_ADD_MAKE_CLEAN_FILES};${ECOS_INSTALL_PATH}/ecos/install/lib/target.ld)
SET_DIRECTORY_PROPERTIES(
    PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${ECOS_ADD_MAKE_CLEAN_FILES}" )

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${ECOS_INSTALL_PATH}/ecos/install/lib/target.ld
    COMMAND sh
    ARGS -c \"make -C ecos || exit -1\; if [ -e ${ECOS_INSTALL_PATH}/ecos/install/lib/target.ld ] \; then touch ${ECOS_INSTALL_PATH}/ecos/install/lib/target.ld\; fi\"
    DEPENDS ${ECOS_INSTALL_PATH}/makefile
)

ADD_CUSTOM_COMMAND(OUTPUT ${ECOS_INSTALL_PATH}/makefile
    COMMAND sh
    ARGS -c \" cd ecos\; ${ECOSCONFIG_EXECUTABLE} tree || exit -1\;\"
    DEPENDS ${ECOS_INSTALL_PATH}/ecos.ecc
)

ADD_CUSTOM_TARGET( ecos make -C ${ECOS_INSTALL_PATH}/ DEPENDS  ${ECOS_INSTALL_PATH}/makefile )

SET (_ecos_EXTRA_LIBS "${_ecos_EXTRA_LIBS} -L${EML_BINARY_DIR}/src -leml")

#special link commands for ecos-executables
SET(CMAKE_CXX_LINK_EXECUTABLE  "<CMAKE_CXX_COMPILER> <CMAKE_CXX_LINK_FLAGS> <OBJECTS>  -o <TARGET> ${_ecos_EXTRA_LIBS} -nostdlib  -nostartfiles -L${ECOS_INSTALL_PATH}/ecos/install/lib -Ttarget.ld ${ECOS_LD_MCPU}")
SET(CMAKE_C_LINK_EXECUTABLE  "<CMAKE_C_COMPILER> <CMAKE_C_LINK_FLAGS> <OBJECTS>  -o <TARGET> ${_ecos_EXTRA_LIBS} -nostdlib  -nostartfiles -L${ECOS_INSTALL_PATH}/ecos/install/lib -Ttarget.ld ${ECOS_LD_MCPU}")

ADD_EXECUTABLE(ethercat_master_test.elf ethercat.cxx src/process_data.cxx)

SET_TARGET_PROPERTIES(${_exe_NAME} PROPERTIES SUFFIX ".elf")

# ECOS_ADD_EXECUTABLE(ethercat_master_test.elf ethercat.cxx)

