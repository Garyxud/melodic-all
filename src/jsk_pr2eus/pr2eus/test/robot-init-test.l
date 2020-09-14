#!/bin/env roseus

;; robot-init test

(require :unittest "lib/llib/unittest.l")

(load "package://pr2eus/robot-interface.l")
(ros::roseus "test_robot_init")

(init-unit-test)

;;(deftest test-robot-init-without-param-and-argument ()
;;  (assert (not (robot-init)) "This should invoke error."))

;; disaable test for jade/kinetic, which did not load pr2-interface.l, beacuse of missing pr2_controller_msgs
(when (ros::rospack-find "(pr2_controllers_msgs")

(deftest test-robot-init-with-argument ()
  (assert (progn
            (robot-init "pr2")
            (and *ri* *pr2*))
          "This should return pr2 and ri.")
  )

(deftest test-robot-init-without-argument-with-param ()
  (makunbound '*ri*)
  (makunbound '*pr2*)
  (ros::set-param "/robot/type" "pr2")
  (assert (progn
            (robot-init)
            (and *ri* *pr2*))
          "This should return pr2 and ri.")
  )
)

(run-all-tests)
(exit)

