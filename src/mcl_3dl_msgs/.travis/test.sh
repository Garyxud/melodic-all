#!/bin/bash

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /catkin_ws

pkgs=$(find . -name package.xml | xargs -n1 dirname)
catkin_lint $pkgs \
  || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO}" \
      "<details><summary>catkin_lint failed</summary>

\`\`\`
$(catkin_lint $pkgs 2>&1)
\`\`\`
</details>"; false)


sed -i -e '5a set(CMAKE_C_FLAGS "-Wall -Werror")' \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e '5a set(CMAKE_CXX_FLAGS "-Wall -Werror")' \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

CM_OPTIONS=""

catkin_make ${CM_OPTIONS} \
  || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests ${CM_OPTIONS} \
  || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests ${CM_OPTIONS} \
  || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
then
  result_text="
\`\`\`
$(catkin_test_results --all | grep -v Skipping || true)
\`\`\`
"
else
  result_text="
\`\`\`
$(catkin_test_results --all | grep -v Skipping || true)
\`\`\`
$(find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;')
"
fi
catkin_test_results || (gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] FAILED on ${ROS_DISTRO}" "<details><summary>Test failed</summary>

$result_text</details>"; false)

gh-pr-comment "[#${TRAVIS_BUILD_NUMBER}] PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
