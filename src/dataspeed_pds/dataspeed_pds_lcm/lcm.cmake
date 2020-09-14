cmake_minimum_required(VERSION 2.8.3)

# Find OR install LCM
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(PkgConfig REQUIRED)
find_package(LCM)
if(NOT LCM_FOUND)
  # Extract *.tar.gz files in the 3rdparty folder
  FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.tar.gz")
  FOREACH(filename ${files})
    MESSAGE(STATUS "Extracting file: ${filename}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    )
  ENDFOREACH(filename)
  # Install Debian packages with dpkg
  FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/liblcm1*_amd64.deb")
  STRING(REPLACE ";" " " files "${files}")
  MESSAGE(STATUS "Installing Debian package: ${files}")
  execute_process(
    COMMAND sudo dpkg -i ${files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
  )
  # Install Debian packages with dpkg
  FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/liblcm-bin*_amd64.deb")
  STRING(REPLACE ";" " " files "${files}")
  MESSAGE(STATUS "Installing Debian package: ${files}")
  execute_process(
    COMMAND sudo dpkg -i ${files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
  )
  # Install Debian packages with dpkg
  FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/liblcm-dev*_amd64.deb")
  STRING(REPLACE ";" " " files "${files}")
  MESSAGE(STATUS "Installing Debian package: ${files}")
  execute_process(
    COMMAND sudo dpkg -i ${files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
  )
endif()
find_package(LCM REQUIRED)
