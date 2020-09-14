(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/robot-interface.l")
(load "irteus/demo/sample-robot-model.l")

(init-unit-test)

(defclass sample-robot-interface
  :super robot-interface
  :slots ())
(defmethod sample-robot-interface
  (:init
   (&rest args)
   (send-super* :init :robot sample-robot args)
   self))

(when (not (boundp '*ri*))
  (setq *robot* (instance sample-robot :init))
  (setq *ri* (instance sample-robot-interface :init)))


(deftest test-angle-vector
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  )

(deftest test-end-coords-interpolation
  (assert (send *robot* :reset-pose))
  (assert (send *robot* :larm :move-end-pos #f(50 0 0) :world))
  (assert (send *robot* :rarm :move-end-pos #f(50 0 0) :world))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 1000 nil 0
                :end-coords-interpolation t))
  )

(deftest test-wait-interpolation
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (null (some #'identity (send *ri* :wait-interpolation))))
  )

(deftest test-wait-interpolation-smooth
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (null (some #'identity (send *ri* :wait-interpolation-smooth 1000))))
  )

(deftest test-state
  (assert (null (send *ri* :state)))
  )

(deftest test-potentio-vector
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (null (some #'identity (send *ri* :wait-interpolation))))
  (assert (< (norm (v- (send *robot* :angle-vector) (send *ri* :potentio-vector))) 10.0))
  )

(deftest test-state-wait-until-update
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (send *ri* :state :potentio-vector))
  (assert (send *ri* :state :potentio-vector :wait-until-update t))
  )

(deftest test-worldpos
  (assert (send *ri* :worldcoords))
  )

(deftest test-objects
  (load "models/room610-scene.l")
  (room610)
  (setq *ri* (instance sample-robot-interface :init :objects (send *room610* :objects)))
  (assert (send *ri* :objects) "robot-interface :objects")
  (setq *laundry* (send *room610* :object "room610-laundry-machine"))
  (assert (string= (send (car (send *ri* :find-object "room610-laundry-machine")) :name)
                   (send *laundry* :name))
          "robot-interface :find-object by name")
  (assert (string= (send (car (send *ri* :find-object *laundry*)) :name)
                   (send *laundry* :name))
          "robot-interface :find-object by object of same name")
  )

;; this should be run in the last
(deftest test-objects-no-display
  (setq x::*display* nil)
  (setq *ri* (instance sample-robot-interface :init :objects (send *room610* :objects)))
  (assert (send *ri* :objects) "robot-interface :objects")
  )

(run-all-tests)
(exit)



