^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_hand_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.8 (2020-03-25)
------------------
* remove -Wno-format flag
* Contributors: fmessmer

0.6.7 (2020-03-18)
------------------
* Merge pull request `#26 <https://github.com/ipa320/cob_hand/issues/26>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#25 <https://github.com/ipa320/cob_hand/issues/25>`_ from fmessmer/suppress_warnings
  add -Wno-* compile option to suppress warnings
* add -Wno compile option to suppress warnings
* Contributors: Felix Messmer, fmessmer

0.6.6 (2019-08-06)
------------------

0.6.5 (2018-07-22)
------------------
* update pigpio submodule
* Contributors: fmessmer

0.6.4 (2018-07-21)
------------------
* update maintainer
* Merge pull request `#22 <https://github.com/ipa320/cob_hand/issues/22>`_ from ipa-bnm/fix/cob_hand_init
  Reset error in init
* reset error on init
* readded local launch file
* Merge pull request `#21 <https://github.com/ipa320/cob_hand/issues/21>`_ from ipa-bnm/feature/direct_sdh_connection
  added ros node for direct sdax communication
* mojin robotics copyright
* remove launch file
* added ros node for direct sdax communication
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, fmessmer, ipa-fmw

0.6.3 (2018-01-07)
------------------
* Merge pull request `#19 <https://github.com/ipa320/cob_hand/issues/19>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#17 <https://github.com/ipa320/cob_hand/issues/17>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* change maintainer
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.2 (2017-07-18)
------------------
* properly handle re-init after restart
  tested on cob4-5
* Contributors: Mathias Lüdtke

0.6.1 (2016-10-10)
------------------
* fix install tags
* move launch files to bringup
* start init server after status topic is connected
* speed-up line parsing
* fixed offset calculation bug
* stop timer on preemption
* improved deadline timer
* fixed typo in JointValues.msg
* resend while moving
* set nominal current as default
* renamed g_motors_stopped to to g_motion_stopped
* resend only if control was stopped
* set default velocity (unused, but must be in valid range)
* move command string format was wrong, updated to signed
* pass currents to SDHx
* read currents from trajectory
* parse default currents
* non-positive resend_timer disables it
* stop resending on error
* use deadline timer instead of resend timer
* properly handle goal if already at target
* added missing dependency
* Fix for `#6 <https://github.com/ipa320/cob_hand/issues/6>`_, added dependency to self
* deep-copy port string
* require movement to start
* migrated to goalCB
* increased stopped velocity
* support trajectories with missing header stamp
* proper conversions
* do not test for tolerances size (outdated code)
* catch not connected error in init
* first status detected was buggy
* finger ready  logic was inverted
* added trajectory interface
* wait for init_finger service
* enable recover on non-zero RC
* Use global namespaces to make remap work
* Do not check required interfase per default
  rosserial checks FQN, does not work with component namespaces
* report error on non-zero SDHx return code
* added diagnostics
* c&p bug
* device port was missing the leading slash
* resize position vector before filling it
* advertise recover service
* updated mismatched MD5 sums
* added first version of node
* implemented recover service
* actually read velocity and current from SDHx
* switched to signed current entry
* improved line parsing
* Keep existing PWM setting if value is zero
* Create README.md
* PWM just works for first bank
* added launch file
* refactored for PWM feature
* added support for bank1
* removed sdhx.cpp from CMake config
* SDHx implementation
* first test app with GPIO
* Contributors: Florian Weisshardt, Mathias Lüdtke, ipa-cob4-5, ipa-nhg
