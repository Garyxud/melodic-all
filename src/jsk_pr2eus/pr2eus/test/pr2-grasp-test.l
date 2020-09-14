(require :unittest "lib/llib/unittest.l")
(init-unit-test)
(load "package://pr2eus/pr2-interface.l")
(setq *pr2* (pr2))

(setq *ri* (instance pr2-interface :init))

(deftest test-start-grasp ()
  (send *ri* :start-grasp)
  )

(deftest test-start-grasp-with-ri ()
  (send *pr2* :reset-pose)
  (send *pr2* :rarm :move-end-pos #f(0  170 400) :world :rotation-axis nil)
  (send *pr2* :larm :move-end-pos #f(0 -170 400) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :start-grasp)
  )

(deftest test-stop-grasp ()
  (send *ri* :stop-grasp)
  )

(test-start-grasp)
(test-stop-grasp)
(test-start-grasp-with-ri)
(run-all-tests)
(exit)

