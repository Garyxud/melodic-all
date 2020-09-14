# give the project a name, so that the KDevelop project gets a nice name
PROJECT(helloworld-i386)

#include the file with the source list
INCLUDE(../ProjectSources.txt)

#compile for the i386 processor
ECOS_USE_I386_ELF_TOOLS()

#add some compile switches
ADD_DEFINITIONS(-O2 -g)

# verbose Makefiles, so that we see the complete compile commands
SET(CMAKE_VERBOSE_MAKEFILE ON)

# we are one directory level deeper here, so the
# relative paths from the ProjectSources.txt have to be adjusted accordingly
ECOS_ADJUST_DIRECTORY(target_SRCS ${PROJECT_SRCS})

#add the eCos include directories
ECOS_ADD_INCLUDE_DIRECTORIES()

#create the eCos executable
ECOS_ADD_EXECUTABLE(${ECOS_PROJECT_NAME} ${target_SRCS})
