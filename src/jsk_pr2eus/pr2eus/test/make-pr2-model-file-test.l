(require :unittest "lib/llib/unittest.l")

(load "package://pr2eus/make-pr2-model-file")
(ros::roseus "make-pr2-modle-file")
(make-pr2-model-file :output-directory "/tmp")

(init-unit-test)

(when (not (boundp '*pr2-unstable*))
  (load "/tmp/pr2.l")
  (setq *pr2-unstable* (pr2))
  (setq *pr2-cameras-unstable* (send *pr2-unstable* :cameras)))
(when (not (boundp '*pr2-stable*))
  (load "package://pr2eus/pr2.l")
  (setq *pr2-stable* (pr2))
  (setq *pr2-cameras-stable* (send *pr2-stable* :cameras)))

(deftest test-angle-vector
  (assert (= (norm (v- (send *pr2-unstable* :angle-vector)
		       (send *pr2-stable* :angle-vector))) 0.0)
	  "check default angle-vector")
  (assert (= (norm (v- (send *pr2-unstable* :reset-pose)
		       (send *pr2-stable* :reset-pose))) 0.0)
	  "check reset-pose angle-vector")
  (send *pr2-unstable* :angle-vector (send *pr2-stable* :angle-vector)))

(deftest test-link ()
  (assert (=
	   (length (send *pr2-unstable* :links))
	   (length (send *pr2-stable* :links)))
	  "check number of link"
	  )
  (mapcar #'(lambda (link-unstable link-stable)
              (warning-message 3 "unstable ~A~%  stable ~A~%" link-unstable link-stable)
	      (assert (equal (send link-unstable :name) (send link-stable :name))
		      (format nil "check link name ~A ~A" (send link-unstable :name) (send link-stable :name)))
	      (assert
	       (eps= (norm (send (send link-unstable :worldcoords) :difference-position (send link-stable :worldcoords))) 0.0)
	       (format nil "check link position ~A ~A" (send link-unstable :worldcoords) (send link-stable :worldcoords)))
	      (assert
	       (eps= (norm (send (send link-unstable :worldcoords) :difference-rotation (send link-stable :worldcoords))) 0.0)
	       (format nil "check link orientation ~A ~A" link-unstable link-stable))
              (unless (eps= (send link-unstable :weight) (send link-stable :weight))
		(ros::ros-warn "check link weight ~16,12f ~16,12f" (send link-unstable :weight) (send link-unstable :weight)))
	      (assert
	       (eps= (send link-unstable :weight) (send link-stable :weight) 0.5)
	       (format nil "check link weight ~A(~A) ~A(~A)" link-unstable (send link-unstable :weight) link-stable (send link-stable :weight))))
	  (send *pr2-unstable* :links) (send *pr2-stable* :links))
  )

(deftest test-joint
  (assert (=
	   (length (send *pr2-unstable* :joint-list))
	   (length (send *pr2-stable* :joint-list)))
	  "check number of joint")

  (mapcar #'(lambda (joint-unstable joint-stable)
	      (assert (string-equal (send joint-unstable :name) (send joint-stable :name))
		      (format nil "check joint name ~A ~A" (send joint-unstable :name) (send joint-stable :name)))
	      ;;

	      (unless (= (send joint-unstable :min-angle)  (send joint-stable :min-angle))
		(warning-message 3 "check min angle between unstable ~16,12f and stable ~16,12f of ~A~%" (send joint-unstable :min-angle) (send joint-stable :min-angle) (send joint-stable :name) ))
	      (unless (= (send joint-unstable :max-angle)  (send joint-stable :max-angle))
		(warning-message 3 "check max angle between unstable ~16,12f and stable ~16,12f of ~A~%" (send joint-unstable :max-angle) (send joint-stable :max-angle) (send joint-stable :name) ))
	      ;;
	      (unless
	       (<= (- (send joint-unstable :min-angle)  (send joint-stable :min-angle)) 0)
	       (warning-message 3 "check min angle between unstable ~16,12f >= stable ~16,12f of ~A~%" (send joint-unstable :min-angle) (send joint-stable :min-angle) (send joint-stable :name) ))
	      (unless
	       (>= (- (send joint-unstable :max-angle)  (send joint-stable :max-angle)) 0)
	       (warning-message 3 "check max angle between unstable ~16,12f >= stable ~16,12f of ~A~%" (send joint-unstable :max-angle) (send joint-stable :max-angle) (send joint-stable :name) ))
	      (assert
	       (eps= (- (send joint-unstable :min-angle)  (send joint-stable :min-angle)) 0.001)
	       (format nil "check min angle between unstable ~16,12f and stable ~16,12f of ~A" (send joint-unstable :min-angle) (send joint-stable :min-angle) (send joint-stable :name) ))
	      (assert
	       (eps= (- (send joint-unstable :max-angle)  (send joint-stable :max-angle)) 0.001)
	       (format nil "check max angle between unstable ~16,12f and stable ~16,12f of ~A" (send joint-unstable :max-angle) (send joint-stable :max-angle) (send joint-stable :name) ))
	      )
	  (send *pr2-unstable* :joint-list) (send *pr2-stable* :joint-list))
  )


(deftest test-end-coords
  (dolist (limb (list :larm :rarm :head))
    (unless (= (norm (send (send *pr2-unstable* limb :end-coords) :difference-position
			   (send *pr2-stable* limb :end-coords))) 0)
      (ros::ros-warn "strict check end-coords position of ~A assertion!" limb)
      (ros::ros-warn "strict check end-coords position : ~A ~A"
		      (send *pr2-unstable* limb :end-coords)
		      (send *pr2-stable* limb :end-coords)))
    (unless (= (norm (send (send *pr2-unstable* limb :end-coords) :difference-rotation
			   (send *pr2-stable* limb :end-coords))) 0)
      (ros::ros-warn "strict check end-coords rotation of ~A assertion!" limb)
      (ros::ros-warn "strict check end-coords rotation : ~A ~A"
		      (send *pr2-unstable* limb :end-coords)
		      (send *pr2-stable* limb :end-coords)))
    (assert (eps= (norm (send (send *pr2-unstable* limb :end-coords) :difference-position
			      (send *pr2-stable* limb :end-coords))) 0)
	    (format nil "check end-coords position of ~A" limb))
    (assert (eps= (norm (send (send *pr2-unstable* limb :end-coords) :difference-rotation
			      (send *pr2-stable* limb :end-coords))) 0)
	    (format nil "check end-coords rotation of ~A" limb))
    ))

(deftest test-jacobian
  (dolist (limb (list :larm :rarm :head))
    (assert
     (eps=
      (norm
       (array-entity
	(m-
	 (send *pr2-unstable* :calc-jacobian-from-link-list
	       (send *pr2-unstable* :link-list (send *pr2-unstable* limb :end-coords :parent))
	       :move-target (send *pr2-unstable* limb :end-coords))
	 (send *pr2-stable* :calc-jacobian-from-link-list
	       (send *pr2-stable* :link-list (send *pr2-stable* limb :end-coords :parent))
	       :move-target (send *pr2-stable* limb :end-coords))))
       )
      0.0)
     (format nil "check jacobian for ~A~%" limb))))

(deftest test-cameras
  (unless (= (length *pr2-cameras-unstable*)
	     (length *pr2-cameras-stable*))
    (ros::ros-warn "strict check camera number unstable:~A stable:~A" (length *pr2-cameras-unstable*) (length *pr2-cameras-stable*)))
  (assert (>= (length *pr2-cameras-unstable*)
	      (length *pr2-cameras-stable*))
	  "check camera number")
  (dolist (camera-stable *pr2-cameras-stable*)
    (let* ((camera-name (send camera-stable :name))
	   (camera-unstable (find camera-name *pr2-cameras-unstable* :key #'(lambda (x) (send x :name)) :test #'(lambda (a b) (string-equal (string a) (string b))))))
      (assert camera-unstable
	      (format nil "check camera ~A" camera-name))
      (when (and camera-unstable (>= (elt (unix::getenv "ROS_DISTRO") 0) #\i)) ;; if distro is <= hydro could not pass the test
	(unless (= (norm (send camera-unstable :difference-position
			     camera-stable)) 0.0)
		(warning-message 3 "check camera position for ~A ~A ~A ~16,13f~%" camera-name
			camera-unstable camera-stable
			(norm (send camera-unstable :difference-position
				    camera-stable))))
	(assert (eps= (norm (send camera-unstable :difference-position
			     camera-stable)) 0.0 1.0)
		(format nil "check camera position for ~A ~A ~A ~16,13f" camera-name
			camera-unstable camera-stable
			(norm (send camera-unstable :difference-position
				    camera-stable))))
	(unless (= (norm (send camera-unstable :difference-rotation
			       camera-stable)) 0.0)
		(warning-message 3 "check camera rotation for ~A ~A ~A ~16,13f~%" camera-name
			camera-unstable camera-stable
			(norm (send camera-unstable :difference-rotation
				    camera-stable))))
	(assert (eps= (norm (send camera-unstable :difference-rotation
			       camera-stable)) 0.0 (deg2rad 1.0))
		(format nil "check camera rotation for ~A ~A ~A ~16,13f" camera-name
			camera-unstable camera-stable
			(norm (send camera-unstable :difference-rotation
				    camera-stable))))
	(assert (= (send camera-unstable :width)
		   (send camera-stable :width))
		(format nil "check camera width for ~A" camera-name))
	(assert (= (send camera-unstable :height)
		   (send camera-stable :height))
		(format nil "check camera height for ~A" camera-name))
	(assert (= (send camera-unstable :fovy)
		   (send camera-stable :fovy))
		(format nil "check camera fovy for ~A" camera-name))
	(unless (= (norm (v- (send camera-unstable :view-direction)
			     (send camera-stable :view-direction))) 0.0)
		(warning-message 3 "check camera view-direction for ~A ~A ~A ~16,13f~%" camera-name
			(send camera-unstable :view-direction)
			(send camera-stable :view-direction)
			(norm (v- (send camera-unstable :view-direction)
				  (send camera-stable :view-direction)))))
	(assert (eps= (norm (v- (send camera-unstable :view-direction)
			     (send camera-stable :view-direction))) 0.0)
		(format nil "check camera view-direction for ~A ~A ~A ~16,13f" camera-name
			(send camera-unstable :view-direction)
			(send camera-stable :view-direction)
			(norm (v- (send camera-unstable :view-direction)
				  (send camera-stable :view-direction)))))
	(assert (eps= (norm
		    (array-entity
		     (m- (send camera-unstable :projection)
			 (send camera-stable :projection)))) 0.0 0.01)
		(format nil "check camera projection for ~A ~A ~A ~A" camera-name
			(send camera-unstable :projection)
			(send camera-stable :projection)
			(norm
			 (array-entity
			  (m- (send camera-unstable :projection)
			      (send camera-stable :projection))))))
	)))
  )

(run-all-tests)

(exit)
