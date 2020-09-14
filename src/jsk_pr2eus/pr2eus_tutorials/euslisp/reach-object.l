(load "package://pr2eus_tutorials/euslisp/robot-setup.l")

(ros::load-ros-manifest "jsk_recognition_msgs")

(ros::roseus "reach_object_demo")

(defun setup
  ()
  (setq *topic-name* "/HSI_color_filter/boxes")
  (setq *bbox-coords* nil)
  (ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bbox-array-cb 1)
  (setq *tfl* (instance ros::transform-listener :init))
  )

(defun send-look-pose
  ()
  (send *robot* :reset-manip-pose)
  (send *robot* :head :neck-p :joint-angle 35)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun bbox-array-cb (msg)
  (let* ((recog-coords (ros::tf-pose->coords (send (car (send msg :boxes)) :pose)))
         (trans-coords (send *tfl* :lookup-transform (send (car (send *robot* :links)) :name) (send msg :header :frame_id) (ros::time 0))))
    (setq *bbox-coords* (send trans-coords :transform recog-coords))
    ))

(defun wait-until-new-bbox
  ()
  (setq *bbox-coords* nil)
  (ros::rate 10)
  (while (not *bbox-coords*)
    (ros::spin-once)
    (ros::sleep)
    )
  (warn ";; *bbox-coords* is ~a~%" *bbox-coords*)
  )

(defun go-pos-to-bbox
  ()
  (let* ((target-pos (coerce (scale 1e-3 (v+ (send *bbox-coords* :pos) (float-vector -500 0 0))) cons)))
    (cond ((member :go-pos-unsafe (send *ri* :methods))
           (send* *ri* :go-pos-unsafe target-pos))
          (t
           (send* *ri* :go-pos target-pos)))
    ))

(defun send-reach-pose
  ()
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :larm :inverse-kinematics
        (make-coords :pos (send (send (send (car (send *robot* :links)) :copy-worldcoords) :transform *bbox-coords*) :worldpos))
        :rotation-axis :z
        :revert-if-fail nil
        :debug-view :no-message)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun reach-object-demo
  ()
  (warn "Generate *robot* and *ri* before starting demo~%")
  (setup)
  (warn "Press Enter to send look pose~%")
  (read-line)
  (send-look-pose)
  (warn "Press Enter to wait until new bbox~%")
  (read-line)
  (wait-until-new-bbox)
  (warn "Press Enter to go pos to object~%")
  (read-line)
  (send *bbox-coords* :draw-on :flush t :size 50 :width 5 :color #f(1 0 0))
  (go-pos-to-bbox)
  (warn "Press Enter to wait until new bbox~%")
  (read-line)
  (wait-until-new-bbox)
  (warn "Press Enter to send reaching pose~%")
  (read-line)
  (send-reach-pose)
  )

(warn "(reach-object-demo)~%")
