(load "unittest.l")
(load "package://euscollada/pr2.l")
;;(load "package://pr2eus/pr2-utils.l")

(init-unit-test)

;; (deftest pr2-test ()
;;   (if (not (boundp '*pr2*)) (pr2))
;;   (send *pr2* :move-to (make-coords) :world)
;;   (send *pr2* :reset-pose)
;;   (if (eq x::*display* 0) (setq x::*display* nil))
;;   (if x::*display* (objects (list *pr2*)))
;;   (dotimes (i 100)
;;    (print (list i (send *pr2* :torso :waist-z :joint-angle)))
;;    (setq p (make-coords :pos
;;                         (v+ (float-vector 400 -400 1000)
;;                             (float-vector
;;                              0
;;                              (* 300 (sin (* pi (/ i 25.0))))
;;                              (* 500 (sin (* pi (/ i 50.0))))))))
;;    (assert
;;     (send *pr2* :inverse-kinematics p
;; 	  :look-at-target t
;; 	  :rotation-axis :z
;; 	  :use-base 0.1
;; 	  )
;;     (format nil "check ik for ~A" p))
;;    (when x::*display*
;;      (send *irtviewer* :objects (list *pr2* p))
;;      (send *irtviewer* :draw-objects)
;;      (x::window-main-one))
;;    )
;;   (if x::*display*
;;     (send-all (send *pr2* :links) :draw-on :flush t))
;;   )

(deftest pr2-weight-test
  (if (not (boundp '*pr2*)) (pr2))
  (let* ((strm (piped-fork "echo $(grep \"<mass\" `rospack find pr2_mechanism_model`/pr2.urdf | cut -d\\\" -f2)"))
	 (urdf-weight (reduce #'+ (read-from-string (format nil "(~A )" (read-line strm nil nil)))))
	 (euslisp-weight (* 1e-3 (send *pr2* :weight)))) ;; [g] -> [kg]
    (close strm)
    (assert (eps= urdf-weight euslisp-weight) (format nil "check weight urdf:~A euslisp:~A" urdf-weight euslisp-weight))))

(load "irteus/irtmodel.l")
(deftest pr2-eusmodel-validity-check
  (eusmodel-validity-check-one *pr2*))

(run-all-tests)
(exit)
