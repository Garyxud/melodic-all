#!/bin/bash

set -o errexit
set -o verbose

scriptdir=`dirname "${0}"`

pip install gh-pr-comment

source /opt/ros/${ROS_DISTRO}/setup.bash

cd /catkin_ws

if [ -f $scriptdir/deps.rosinstall ];
then
  wstool init src $scriptdir/deps.rosinstall
fi

SKIP_KEYS=
if [ -f $scriptdir/rosdepkeys.skip ];
then
  SKIP_KEYS="--skip-keys=\"$(cat $scriptdir/rosdepkeys.skip | tr '\n' ' ' | sed 's/\s*$//g')\""
fi

echo $SKIP_KEYS

apt-get -qq update
apt-get install libxml2-utils
eval rosdep install --from-paths src --ignore-src $SKIP_KEYS --rosdistro=${ROS_DISTRO} -y
apt-get clean
rm -rf /var/lib/apt/lists/*

CMI_OPTION="--install-space /opt/ros/${ROS_DISTRO} --install"
CMI_ONLY_PKG="--pkg ${PACKAGE_NAME}"

catkin_make_isolated $CMI_OPTION || \
  (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make_isolated $CMI_OPTION $CMI_ONLY_PKG --catkin-make-args tests || \
  (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make_isolated $CMI_OPTION $CMI_ONLY_PKG --catkin-make-args run_tests || \
  (gh-pr-comment "FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
then
  result_text="
\`\`\`
`catkin_test_results --all | grep -v Skipping || true`
\`\`\`
"
else
  result_text="
\`\`\`
`catkin_test_results --all || true`
\`\`\`
`find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;'`
"
fi
catkin_test_results || (gh-pr-comment "FAILED on ${ROS_DISTRO}" "Test failed$result_text"; false)

gh-pr-comment "PASSED on ${ROS_DISTRO}" "All tests passed$result_text"

cd ..
rm -rf /catkin_ws || true
