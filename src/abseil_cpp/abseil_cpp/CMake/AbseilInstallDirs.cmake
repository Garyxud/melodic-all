include(GNUInstallDirs)

# absl_VERSION is only set if we are an LTS release being installed, in which
# case it may be into a system directory and so we need to make subdirectories
# for each installed version of Abseil.  This mechanism is implemented in
# Abseil's internal Copybara (https://github.com/google/copybara) workflows and
# isn't visible in the CMake buildsystem itself.

set(ABSL_INSTALL_BINDIR "${CATKIN_PACKAGE_BIN_DESTINATION}")
set(ABSL_INSTALL_CONFIGDIR "${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}")
set(ABSL_INSTALL_INCLUDEDIR "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
set(ABSL_INSTALL_LIBDIR "${CATKIN_PACKAGE_LIB_DESTINATION}")
