#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::load-ros-manifest "pr2eus_moveit")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

;; avoid print violate max/min-angle that exceeds 4M log limit
(unless (assoc :joint-angle-org (send rotational-joint :methods))
        (rplaca (assoc :joint-angle (send rotational-joint :methods)) :joint-angle-org))
(defmethod rotational-joint
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
	      (setq v max-angle)))
       (cond ((< v min-angle)
	      (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  )

;; wait for gazebo
(ros::roseus "test_pr2eus_mvoeit_client")
(ros::advertise "head_traj_controller/joint_trajectory_action/goal" pr2_controllers_msgs::JointTrajectoryActionGoal)
(while (= (ros::get-num-subscribers "head_traj_controller/joint_trajectory_action/goal") 0)
  (unix::sleep 1)
  (format *error-output* "waiting for controller ... head_traj_controller (~A)~%" (ros::time-now)))

(defmethod pr2-interface
  (:fullbody-2-controller ()
    (append
     (send self :rarm-controller)
     (send self :larm-controller)
     (send self :head-controller)
     (send self :torso-controller)))
  (:rarm-head-controller ()
    (append
     (send self :rarm-controller)
     (send self :head-controller)))
  (:larm-head-controller ()
    (append
     (send self :larm-controller)
     (send self :head-controller)))
  (:dummy-head-controller ()
    (send self :head-controller))
  (:dummy-2-head-controller ()
   ()
   (list
	(list
	 (cons :controller-action "head_traj_controller/follow_joint_trajectory")
	 (cons :controller-state "head_traj_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:dummy-rarm-controller ()
    (send self :rarm-controller))
  (:dummy-2-rarm-controller ()
   ()
   (list
	(list
	 (cons :controller-action "r_arm_controller/follow_joint_trajectory")
	 (cons :controller-state "r_arm_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "r_shoulder_pan_joint"
							  "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
							  "r_elbow_flex_joint" "r_forearm_roll_joint"
							  "r_wrist_flex_joint" "r_wrist_roll_joint")))))
  )

;; setup pr2 and ri
;;(setq *ri* (instance pr2-interface :init :type :rarm-head-controller))
(pr2-init)

;;
(send *ri* :add-controller :fullbody-2-controller :create-actions t)
(send *ri* :add-controller :rarm-head-controller :create-actions t)
(send *ri* :add-controller :larm-head-controller :create-actions t)
(send *ri* :add-controller :dummy-head-controller :create-actions t)
(send *ri* :add-controller :dummy-2-head-controller :create-actions t)

;; setup moveit interface
(send *ri* :set-moveit-environment (instance pr2-moveit-environment :init))

;; test concatenation of moveit response in angle-vector-make-trajectory
(deftest test-avs-to-angle-vector-make-trajectory ()
  (let (motion-plan-res tm-list sorted-tm-list)
    (setq motion-plan-res (send *ri* :angle-vector-make-trajectory
                                (list (send *pr2* :reset-manip-pose) (send *pr2* :reset-pose))
                                :move-arm :rarm :use-torso t))
    (setq tm-list (mapcar #'(lambda (pt) (send (send pt :time_from_start) :to-sec))
                          (send motion-plan-res :trajectory :joint_trajectory :points)))
    (setq sorted-tm-list (copy-object tm-list))
    (sort sorted-tm-list #'<=)
    (assert (equal tm-list sorted-tm-list) "time_from_start is not arranged in ascending order")
    ))

;; test trajectory-filter about velocities and accelerations
(deftest test-trajectory-filter-vel-acc ()
  (let ((traj (instance moveit_msgs::RobotTrajectory :init))
        (jt-traj (instance trajectory_msgs::JointTrajectory :init))
        (jt-traj-pt (instance trajectory_msgs::JointTrajectoryPoint :init))
        (mdof-traj (instance trajectory_msgs::MultiDOFJointTrajectory :init))
        (mdof-traj-pt (instance trajectory_msgs::MultiDOFJointTrajectoryPoint :init))
        (trans (instance geometry_msgs::Transform :init))
        (twist (instance geometry_msgs::Twist :init))
        (orig-val 1.0) (init-tm 1000.0) (total-tm 5000.0) (stamp (ros::time-now))
        ret-traj time-scale res-vel res-acc)
    ;; make dummy joint trajectory
    (send jt-traj-pt :positions (float-vector orig-val))
    (send jt-traj-pt :velocities (float-vector orig-val))
    (send jt-traj-pt :accelerations (float-vector orig-val))
    (send jt-traj-pt :time_from_start (ros::time (/ init-tm 1000)))
    (send jt-traj :points (list jt-traj-pt))
    (send jt-traj :joint_names (list "test_jt"))
    (send jt-traj :header :stamp stamp)
    (send traj :joint_trajectory jt-traj)
    ;; make dummy multi dof joint trajectory
    (send trans :translation :x orig-val)
    (send trans :translation :y orig-val)
    (send trans :translation :z orig-val)
    (send trans :rotation :x orig-val)
    (send trans :rotation :y orig-val)
    (send trans :rotation :z orig-val)
    (send trans :rotation :w orig-val)
    (send twist :linear :x orig-val)
    (send twist :linear :y orig-val)
    (send twist :linear :z orig-val)
    (send twist :angular :x orig-val)
    (send twist :angular :y orig-val)
    (send twist :angular :z orig-val)
    (send mdof-traj-pt :transforms (list trans))
    (send mdof-traj-pt :velocities (list (copy-object twist)))
    (send mdof-traj-pt :accelerations (list (copy-object twist)))
    (send mdof-traj-pt :time_from_start (ros::time (/ init-tm 1000)))
    (send mdof-traj :points (list mdof-traj-pt))
    (send mdof-traj :joint_names (list "test_jt"))
    (send mdof-traj :header :stamp stamp)
    (send traj :multi_dof_joint_trajectory mdof-traj)
    ;; process dummy traj with trajectory-filter
    (setq ret-traj (send *ri* :trajectory-filter traj :total-time total-tm :clear-velocities nil))
    ;; check joint trajectory
    (setq time-scale (/ total-tm init-tm))
    (assert (eps= (aref (send (car (send ret-traj :joint_trajectory :points)) :velocities) 0)
                  (/ orig-val time-scale))
            "velocities are not scaled")
    (assert (eps= (aref (send (car (send ret-traj :joint_trajectory :points)) :accelerations) 0)
                  (/ orig-val (expt time-scale 2)))
            "accelerations are not scaled")
    ;; check multi dof joint trajectory
    (dolist (twist-key (list :linear :angular))
      (dolist (vec-key (list :x :y :z))
        (push
          (eps= (send (car (send (car (send ret-traj :multi_dof_joint_trajectory :points))
                                 :velocities))
                      twist-key vec-key)
                (/ orig-val time-scale))
          res-vel)
        (push
          (eps= (send (car (send (car (send ret-traj :multi_dof_joint_trajectory :points))
                                 :accelerations))
                      twist-key vec-key)
                (/ orig-val (expt time-scale 2)))
          res-acc)))
    (assert (null (member nil res-vel)) "velocities of multi dof traj are not scaled")
    (assert (null (member nil res-acc)) "accelerations of multi dof traj are not scaled")
    ))

;; send angle-vector with collision avoidance
(deftest test-angle-vector-motion-plan ()
  (let (av-diff)
    (send *ri* :angle-vector-motion-plan (send *pr2* :reset-pose) :move-arm :rarm :use-torso t)
    (send *ri* :angle-vector-motion-plan (send *pr2* :reset-pose) :move-arm :larm :use-torso t)
    (send *ri* :wait-interpolation)
    (setq av-diff (norm (apply #'float-vector (mapcar #'(lambda (x y) (let ((d (- x y))) (- d (* 360 (round (/ d 360)))))) (coerce (send *ri* :state :potentio-vector) cons) (coerce (send *pr2* :reset-pose) cons)))))
    (ros::ros-info "av diff ~A" av-diff)
    ;; not sure why, but sometimes utf-8 code is displayed and brakes catkin build
    (assert (eps= av-diff 0 5) );(format nil "send reset-pose ~A" av-diff))
    ))

;; send angle-vector-sequence with collision avoidance
(deftest test-angle-vector-sequence-motion-plan ()
  (let (av-diff)
    (send *ri* :angle-vector-motion-plan (list (send *pr2* :init-pose) (send *pr2* :reset-pose)) :move-arm :rarm :use-torso t)
    (send *ri* :angle-vector-motion-plan (list (send *pr2* :init-pose) (send *pr2* :reset-pose)) :move-arm :larm :use-torso t)
    (send *ri* :wait-interpolation)
    (setq av-diff
          (norm (apply #'float-vector (mapcar #'(lambda (x y) (let ((d (- x y))) (- d (* 360 (round (/ d 360))))))
                                              (coerce (send *ri* :state :potentio-vector) cons) (coerce (send *pr2* :reset-pose) cons)))))
    (ros::ros-info "av diff ~A" av-diff)
    (assert (eps= av-diff 0 5) );(format nil "send reset-pose ~A" av-diff))
    ))

;; send target coords
(deftest test-move-end-coords-plan ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 750)) :move-arm :rarm)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 500)) :move-arm :larm)
    (send *ri* :wait-interpolation)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 1000)) :move-arm :larm-torso)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "collsion avoidance motion is too fast ~A" tm-diff))
    ))

;; do not send faster command than moveit output
(deftest test-moveit-fastest-trajectory ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *pr2* :reset-pose))
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :angle-vector #f(0 0 0 0 0 0 0))
    (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :total-time 1000 :move-arm :rarm :use-torso nil)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "collsion avoidance motion is too fast ~A" tm-diff))
    ))

;; send angle-vector with start-offset-time
(deftest test-start-offset-time ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *pr2* :reset-pose))
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :move-end-pos #f(100 0 0) :world)
    (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :start-offset-time 3 :total-time 2000 :move-arm :rarm :use-torso nil)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 4) (format nil "start-offset-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 6) (format nil "start-offset-time is considered multiple times. Traj finishes at ~A" tm-diff))
    ))

;; send angle-vector-sequence with start-offset-time
(deftest test-start-offset-time-with-avs ()
  (let (avs tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *pr2* :reset-pose))
    (send *ri* :wait-interpolation)
    (push (send *pr2* :rarm :move-end-pos #f(100 0 0) :world) avs)
    (push (send *pr2* :rarm :move-end-pos #f(100 0 0) :world) avs)
    (setq avs (reverse avs))
    (send *ri* :angle-vector-motion-plan avs :start-offset-time 3 :total-time 2000 :move-arm :rarm :use-torso nil)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 4) (format nil "start-offset-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 7) (format nil "start-offset-time is considered multiple times. Traj finishes at ~A" tm-diff))
    ))

(run-all-tests)
(exit)

