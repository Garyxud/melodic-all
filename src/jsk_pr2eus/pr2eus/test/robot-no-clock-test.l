#!/bin/env roseus

(load "unittest.l")
(load "package://pr2eus/robot-interface.l")
(load "irteus/demo/sample-robot-model.l")

(init-unit-test)

(ros::roseus "robot_no_clock_test")

(deftest pr2eus-no-clock-test ()
  (setq *ri* (instance robot-interface :init :robot (instance sample-robot :init)))
  )

(run-all-tests)
(exit)
