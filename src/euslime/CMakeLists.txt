cmake_minimum_required(VERSION 2.8.3)
project(euslime)

find_package(catkin REQUIRED catkin_virtualenv)
find_package(Git QUIET)

catkin_package()

# Configure files
configure_file(
  ${PROJECT_SOURCE_DIR}/euslime.el.in
  ${PROJECT_SOURCE_DIR}/euslime.el
)

catkin_generate_virtualenv()

# Install files
install(FILES
  README.md
  requirements.txt
# EusLisp files
  slime-toplevel.l
  slime-util.l
# Emacs Lisp files
  euslime-config.el
  euslime.el
# Destination
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  slime-repl-ansi-color/README.md
  slime-repl-ansi-color/LICENSE
  slime-repl-ansi-color/slime-repl-ansi-color.el
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/slime-repl-ansi-color
)

# Install Python
catkin_python_setup()
catkin_install_python(PROGRAMS
  scripts/euslime
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
