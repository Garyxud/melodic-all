;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")
(require :robot-interface "package://pr2eus/robot-interface.l")
(require :speak "package://pr2eus/speak.l")

(unless (ros::rospack-find "pr2_controllers_msgs")
  (cond ((or (string= (unix::getenv "ROS_DISTRO") "indigo")
	     (string< (unix::getenv "ROS_DISTRO") "indigo"))
	 (ros::ros-error "pr2eus/pr2-interface.l: pr2_controllers_msgs is not found"))
	(t
	 (ros::ros-warn "pr2eus/pr2-interface.l: pr2_controllers_msgs is not released on ~A" (unix::getenv "ROS_DISTRO"))
	 (warning-message 1 "exiting without error....")
	 (exit 0))))

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "pr2_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")
(ros::load-ros-manifest "pr2_mechanism_msgs")
(ros::load-ros-manifest "topic_tools")

;;;
;;; pr2 robot interface
;;;


(defmethod pr2-robot
  (:torque-vector
   (&rest args)
   (if args (ros::ros-warn "pr2 torque-vector does not have parameters"))
   (coerce (send-all (send self :joint-list) :joint-torque) float-vector)))

(defclass pr2-interface
  :super robot-move-base-interface
  :slots (r-gripper-action l-gripper-action
                           finger-pressure-origin
                           ))

(defmethod pr2-interface
  (:init
   (&rest args &key (type :default-controller) &allow-other-keys)
   (send-super* :init :robot pr2-robot :type type
                :groupname "pr2_interface"
                args)
   ;; add controllers
   (dolist (l (list
               (cons :larm-controller "l_arm_controller/follow_joint_trajectory")
               (cons :rarm-controller "r_arm_controller/follow_joint_trajectory")
               (cons :head-controller "head_traj_controller/follow_joint_trajectory")
               (cons :torso-controller "torso_controller/follow_joint_trajectory")))
     (let ((type (car l))
           (name (cdr l))
           action)
       (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                             controller-actions))
       (setf (gethash type controller-table) (list action))
       ))
   ;;
   (ros::advertise "j_robotsound" sound_play::SoundRequest 5)
   ;;
   (ros::subscribe "/pressure/r_gripper_motor" pr2_msgs::PressureState
                   #'send self :pr2-fingertip-callback :rarm-pressure :groupname groupname)
   (ros::subscribe "/pressure/l_gripper_motor" pr2_msgs::PressureState
                   #'send self :pr2-fingertip-callback :larm-pressure :groupname groupname)
   (ros::subscribe "/r_gripper_controller/state" pr2_controllers_msgs::JointControllerState
                   #'send self :pr2-gripper-state-callback :rarm :groupname groupname)
   (ros::subscribe "/l_gripper_controller/state" pr2_controllers_msgs::JointControllerState
                   #'send self :pr2-gripper-state-callback :larm :groupname groupname)
   ;;
   (setq r-gripper-action (instance ros::simple-action-client :init
                                    "/r_gripper_controller/gripper_action"
                                    pr2_controllers_msgs::Pr2GripperCommandAction
                                    :groupname groupname))
   (setq l-gripper-action (instance ros::simple-action-client :init
                                    "/l_gripper_controller/gripper_action"
                                    pr2_controllers_msgs::Pr2GripperCommandAction
                                    :groupname groupname))
   ;; wait for pr2-action server (except move_base)
   (dolist (action (list r-gripper-action l-gripper-action))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, pr2-interface is disabled" action)
       (return)))
   t)
  ;;
  (:state (&rest args) ;; overwrite for jsk_maps/pr2
   (case (car args)
     (:worldcoords
      (unless joint-action-enable
	(return-from :state (send self :worldcoords)))
      (send-super :state :worldcoords (or (cadr args) "world")))
     (t
      (send-super* :state args))
     ))
  (:publish-joint-state () ;; overwrite for pr2
   (send-super :publish-joint-state (append (send robot :joint-list) (send robot :larm :gripper :joint-list) (send robot :rarm :gripper :joint-list))))
  ;;
  (:wait-interpolation (&optional (ctype) (timeout 0)) ;; overwrite for pr2, due to some joint is still moving after joint-trajectry-action stops
   (when (send self :simulation-modep) (return-from :wait-interpolation (send-super :wait-interpolation)))
   (ros::ros-info "wait-interpolation debug: start")
   (send-super :wait-interpolation ctype timeout)
   (ros::ros-info "wait-interpolation debug: end")
   (if (eps= timeout 0)
       (while (ros::ok)
         (send self :update-robot-state)
         (when (every #'(lambda(x)(< (abs (send x :joint-velocity))
                                     (if (derivedp x rotational-joint) 0.05 0.001)))
                      (send robot :joint-list))
           (return))))
   (send-all controller-actions :interpolatingp))
  ;;
  ;;
  (:larm-controller
   ()
   (list
	(list
	 (cons :controller-action "l_arm_controller/follow_joint_trajectory")
	 (cons :controller-state "l_arm_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "l_shoulder_pan_joint"
							  "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
							  "l_elbow_flex_joint" "l_forearm_roll_joint"
							  "l_wrist_flex_joint" "l_wrist_roll_joint")))))
  (:rarm-controller
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
  (:head-controller
   ()
   (list
	(list
	 (cons :controller-action "head_traj_controller/follow_joint_trajectory")
	 (cons :controller-state "head_traj_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:torso-controller
   ()
   (list
	(list
	 (cons :controller-action "torso_controller/follow_joint_trajectory")
	 (cons :controller-state "torso_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "torso_lift_joint")))))
  (:default-controller
   ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :head-controller)
    (send self :torso-controller)))
  (:midbody-controller
   ()
   (list
    (list
     (cons :controller-action "midbody_controller/joint_trajectory_action")
     (cons :controller-state "midbody_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_pan_joint"
			      "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
			      "l_elbow_flex_joint" "l_forearm_roll_joint"
			      "l_wrist_flex_joint" "l_wrist_roll_joint"
			      "r_shoulder_pan_joint"
			      "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
			      "r_elbow_flex_joint" "r_forearm_roll_joint"
			      "r_wrist_flex_joint" "r_wrist_roll_joint"
			      "torso_lift_joint")))
    (send self :head-controller)))
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/joint_trajectory_action")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_pan_joint"
			      "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
			      "l_elbow_flex_joint" "l_forearm_roll_joint"
			      "l_wrist_flex_joint" "l_wrist_roll_joint"
			      "r_shoulder_pan_joint"
			      "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
			      "r_elbow_flex_joint" "r_forearm_roll_joint"
			      "r_wrist_flex_joint" "r_wrist_roll_joint"
			      "torso_lift_joint"
			      "head_pan_joint" "head_tilt_joint")))))
  ;;
  (:controller-angle-vector (av tm type) ;; obsolate
   (send self :angle-vector av tm type))
  (:larm-angle-vector (av tm)
   (send self :angle-vector av tm :larm-controller))
  (:rarm-angle-vector (av tm)
   (send self :angle-vector av tm :rarm-controller))
  (:head-angle-vector (av tm)
   (send self :angle-vector av tm :head-controller))
  ;;
  (:move-gripper
   (arm pos &key (effort 25) (wait t) (ignore-stall))
   "Moves gripper of `arm` to target `pos` with `effort`.
    `arm` is either :rarm, :larm or :arms.
    `pos` is the desired distance between grippers [m].
    If wait is T, this function returns T for each arm if reached goal or stalled, NIL otherwise.
    If `ignore-stall` is T, returns NIL for each arm on stall. (e.g. when an object is grasped).
    If wait if NIL, returns T for each arm if goal is accepted, NIL otherwise."
   (unless joint-action-enable
     (send robot arm :gripper :joint-angle (* pos 1000))
     (send self :publish-joint-state)
     (if viewer (send self :draw-objects))
     (return-from :move-gripper t))
   (let* (goal
	  (clients (case arm
		     (:rarm (list r-gripper-action))
		     (:larm (list l-gripper-action))
		     (:arms (list r-gripper-action l-gripper-action))
		     (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
    result)
     (dolist (client clients)
       (setq goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal))
     (setq result
           (if wait
               (progn
                 (send-all clients :wait-for-result)
                 (mapcar #'(lambda (res)
                             (or (if ignore-stall nil (send res :stalled))
                                 (send res :reached_goal)))
                         (send-all clients :get-result)))
               (mapcar #'(lambda (ac)
                           (send ac :spin-once)
                           (not (null
                                 (memq (send ac :get-state)
                                       (list actionlib_msgs::GoalStatus::*PENDING*
                                             actionlib_msgs::GoalStatus::*ACTIVE*
                                             actionlib_msgs::GoalStatus::*SUCCEEDED*)))))
                       clients)))
     (case arm (:arms result) (t (car result)))
     ))
  (:gripper (&rest args)
   "get information of gripper
Arguments:
 - arm (:larm :rarm :arms)
 - type (:position :velocity :pressure)
Example: (send self :gripper :rarm :position) => 0.00"
   (when (eq (car args) :arms)
     (return-from :gripper
       (mapcar #'(lambda (x)
                   (send self :gripper x (cadr args)))
               '(:larm :rarm))))
   (unless (memq (car args) '(:larm :rarm))
     (error "you must specify arm ~A from ~A" (car args) '(:larm :rarm))
     (return-from :gripper nil))
   (send self :state
         (intern
          (format nil "~A-~A" (string (car args)) (string (cadr args)))
          *keyword-package*)))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 0.01) ((:objects objs) objects) force-assoc ((:wait wait) t))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait wait)
   (unless joint-action-enable
     (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
       (let ((grasp-convex
	      (convex-hull-3d
	       (flatten
		(mapcar
		 #'(lambda (l)
		     (send-all (send l :bodies) :worldcoords)
		     (send-all (send l :bodies) :vertices))
		 (send robot a :gripper :links))))))
	 (dolist (obj objs)
	   (when (or force-assoc
               (and (find-method obj :faces)
                    (not (= (pqp-collision-check grasp-convex obj) 0))))
	     (if (send obj :parent) (send (send obj :parent) :dissoc obj))
	     (send robot a :end-coords :assoc obj)))))
     ;; (send self :update-robot-state) ;; update state of 'robot' for real robot
     (return-from :start-grasp
       (case arm
         (:larm (send robot :l_gripper_joint :joint-angle))
         (:rarm (send robot :r_gripper_joint :joint-angle))
         (t (list
             (send robot :l_gripper_joint :joint-angle)
             (send robot :r_gripper_joint :joint-angle))))))
   ;; for real robot
   (let ((clients (case arm
                    (:rarm (list (cons :r_gripper_joint r-gripper-action)))
                    (:larm (list (cons :l_gripper_joint l-gripper-action)))
                    (:arms (list (cons :r_gripper_joint r-gripper-action)
                                 (cons :l_gripper_joint l-gripper-action)))
                    (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
         aresult ajoint (resend-goal t) (resend-count 0) ret)
     (while (and resend-goal (< resend-count 2))
       (setq resend-goal nil)
       (send self :update-robot-state) ;; update state of 'robot' for real robot
       (dolist (client clients)
         (setq aresult (send (cdr client) :get-result)
               ajoint (send robot (car client)))
         (ros::ros-debug ";; :move-gripper check reached_goal ~A, position ~A, robot-body angle ~A" (send aresult :reached_goal) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
         (unless (send aresult :reached_goal) ;; in case that reached_gal is fail, try one more time
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A, unreached" (send ajoint :name) (* 1000 (send aresult :position)))
           (setq resend-goal t))
       ;;
         (unless (eps= (* 1000 (send aresult :position)) (send ajoint :joint-angle) 2)
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A/~A, result and update-body differs" (send ajoint :name) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
           (setq resend-goal t)))
       (incf resend-count)
       (if resend-goal (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait wait)))
     (setq ret
           (case arm
             (:rarm (* 1000 (send (send r-gripper-action :get-result) :position)))
             (:larm (* 1000 (send (send l-gripper-action :get-result) :position)))
             (:arms (mapcar #'(lambda (c) (* 1000 (send (send c :get-result) :position))) (list r-gripper-action l-gripper-action)))))
     (ros::ros-debug ";; :move-gripper ~A returns ~A" (send ajoint :name) ret)
     ret))

  (:get-grasp-result
   (&optional (arm :arms))
   (let (rres lres)
     (send self :spin-once)
     (when (or (eq arm :arms) (eq arm :rarm))
       (when (eq (send r-gripper-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
         (setq rres (send r-gripper-action :get-result)))
       (when (eq (send r-gripper-action :get-state) actionlib_msgs::GoalStatus::*active*)
         (send r-gripper-action :wait-for-result)
         (setq rres (send r-gripper-action :get-result))
         ))
     (when (or (eq arm :arms) (eq arm :larm))
       (when (eq (send l-gripper-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
         (setq lres (send l-gripper-action :get-result)))
       (when (eq (send l-gripper-action :get-state) actionlib_msgs::GoalStatus::*active*)
         (send l-gripper-action :wait-for-result)
         (setq lres (send l-gripper-action :get-result))
         ))
     (setq ret
           (case arm
                 (:rarm (if rres (* 1000 (send rres :position))))
                 (:larm (if lres (* 1000 (send lres :position))))
                 (:arms (mapcar #'(lambda (c) (if c (* 1000 (send c :position))))
                                (list rres lres)))))
     ret
     ))

  (:stop-grasp
   (&optional (arm :arms) &key (wait nil))
   (prog1
       (send self :move-gripper arm 0.09 :wait wait)
     (unless joint-action-enable
       (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
         (dolist (obj (send robot a :end-coords :descendants))
           (send robot arm :end-coords :dissoc obj))))))
  ;;
  (:pr2-gripper-state-callback
   (arm msg)
   (dolist (i (list
               (cons (format nil "~A-POSITION" (string arm))
                     (* 1000.0 (send msg :process_value)))
               (cons (format nil "~A-VELOCITY" (string arm))
                     (* 1000.0 (/ (send msg :process_value_dot) (send msg :time_step))))))
     (send self :set-robot-state1 (intern (car i) *keyword-package*) (cdr i))))
  (:pr2-fingertip-callback
   (arm msg) ;; arm = :(r|l)arm-pressure
   (let ((pressure (list (send msg :l_finger_tip) (send msg :r_finger_tip))))
     (send self :set-robot-state1 arm pressure)))
  (:reset-fingertip
   ()
   (send self :spin-once)
   (setq finger-pressure-origin
	 (mapcar #'(lambda(k)(cons k (copy-seq (send self :state k))))
		 '(:rarm-pressure :larm-pressure))))
  (:finger-pressure
   (arm &key (zero nil))
   (setq arm (case arm (:rarm :rarm-pressure) (:larm :larm-pressure)))
   (let ((current (send self :state arm))
	 (origin (cdr (assoc arm finger-pressure-origin))))
     (if zero
	 (when (and current origin)
	   (mapcar #'v- current origin))
       current)))
  ;;
  (:wait-torso (&optional (timeout 0))
   (let ((act (find-if #'(lambda (x) (string= (send x :name) "torso_controller/follow_joint_trajectory"))
                       controller-actions)))
     (when act
       (send act :wait-for-result :timeout timeout))))
  )


;;
;;
;; workaround for unintentional 360 joint rotation problem [#91]
(defmethod pr2-interface
  (:check-continuous-joint-move-over-180
   (diff-av)
   (let ((i 0) add-new-trajectory-point)
     (dolist (j (send robot :joint-list))
       ;; for continuous rotational joint
       (when (and (> (- (send j :max-angle) (send j :min-angle)) 360)
                  (> (abs (elt diff-av i)) 180))
         (ros::ros-warn "continuous joint (~A) moves ~A degree, commanded joint differs from original trajectory to avoid unintentional 360 rotation" (send j :name) (elt diff-av i))
         (setq add-new-trajectory-point t))
       (incf i (send j :joint-dof)))
     add-new-trajectory-point))
  (:angle-vector
   (av &optional (tm 3000) &rest args)
   (let (diff-av)
     ;; use reference-vector to get last commanded joint and use :angle-vector to toruncate the joint limit to eus lisp style
     (setq diff-av (v- av (send robot :angle-vector (send self :state :reference-vector))))
     ;; use shortest path for contiuous joint
     ;;
     (when (send self :check-continuous-joint-move-over-180 diff-av)
       (return-from :angle-vector
         (send* self :angle-vector-sequence (list av) (list tm) args))) ;; when
     (send-super* :angle-vector av tm args)
     ))
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)) &rest args)
   (unless (or (send self :simulation-modep) (cadr (memq :end-coords-interpolation args)))
     (let* ((prev-av (send robot :angle-vector (send self :state :reference-vector)))
            (len-av (length prev-av))
            (max-av (fill (instantiate float-vector len-av)  180))
            (min-av (fill (instantiate float-vector len-av) -180))
            diff-av (l 0) dist div)
       (dolist (av avs)
         ;; check if joint move more than 180 degree which has -inf/inf limit
         (setq diff-av (v- av prev-av))
         (when (send self :check-continuous-joint-move-over-180 diff-av)
           (setq dist (abs (geo::find-extream (coerce diff-av cons) #'abs #'>=)))
           (setq div (round (/ dist 120.0)))
           (ros::ros-warn "original trajectory command :")
           (ros::ros-warn "                       : ~A ~A" avs tms)
           (ros::ros-warn "  current angle vector : ~A" prev-av)
           (ros::ros-warn "new trajectory command : dvi = ~A" div)
           (dotimes (i div)
             (ros::ros-warn "                       : ~A ~A" (midpoint (/ (+ i 1.0) div) prev-av av) (/ (elt tms l) div)))
           (dotimes (i (1- div))
             (setq avs (list-insert (midpoint (/ (+ i 1.0) div) prev-av av) (+ l i) avs)))
           (setf (elt tms l) (/ (elt tms l) div))
           (dotimes (i (1- div))
             (setq tms (list-insert (elt tms l) l tms)))
           (incf l (1- div))
           (ros::ros-warn "new trajectory command :")
           (ros::ros-warn "                       : ~A ~A" avs tms)
           )
         ;;
         (setq prev-av av)
         (incf l))
       )) ;; when simulation-modep
     (send-super* :angle-vector-sequence avs tms args))
  ) ;; pr2-interface

(defmethod pr2-interface
  (:angle-vector-with-constraint
   (av1 &optional (tm 3000) (arm :arms)
        &key (rotation-axis t) (translation-axis t) (revert-if-fail t) (initial-angle-vector nil) (div 10)
        &rest args)
   "Send trajectory with interpolation in euclidean space.
    `av1` is the target angle vector. `tm` is the total duration of the motion. `arm` is the controller that the trajectory is sent to.
    `:rotation-axis` and `:translation-axis` are for setting constraint on interpolated trajectory.
    If `:revert-if-fail` is NIL, no trajectory is sent if the interpolation is failed.
    If `:initial-angle-vector` is set, the value is used as initial pose of the robot, otherwise the `:reference-vector` is used."
   (let (av0 c0 c1 avs tms ret arm-av pav cur)
     (setq pav (send robot :angle-vector))
     (unless (memq arm '(:larm :rarm :arms))
       (warning-message 1 ":angle-vector-with-constraint arm=~A is not supported, use :larm or :rarm~%" arm)
       (return-from :angle-vector-with-constraint nil))
     (send robot :angle-vector av1)
     (cond ((memq arm '(:larm :rarm))
            (setq c1 (send robot arm :end-coords :copy-worldcoords)))
           (t ;; :arms
            (setq c1 (cons (send robot :larm :end-coords :copy-worldcoords)
                           (send robot :rarm :end-coords :copy-worldcoords)))))
     (if initial-angle-vector
       (setq av0 (send robot :angle-vector initial-angle-vector))
       (setq av0 (send robot :angle-vector (send self :state :reference-vector))))
     (cond ((memq arm '(:larm :rarm))
            (setq c0 (send robot arm :end-coords :copy-worldcoords))
            (setq arm-av (send robot arm :angle-vector)))
           (t
            (setq c0 (cons (send robot :larm :end-coords :copy-worldcoords)
                           (send robot :rarm :end-coords :copy-worldcoords)))
            (setq arm-av (cons (send robot :larm :angle-vector)
                               (send robot :rarm :angle-vector)))))
     (dotimes (i div)
       (setq cur (/ (1+ i) (float div)))
       (send robot :angle-vector (midpoint cur av0 av1))
       (cond ((memq arm '(:larm :rarm))
              (setq arm-av (send robot arm :angle-vector arm-av)))
             (t
              (setq arm-av (list (send robot :larm :angle-vector (car arm-av))
                                 (send robot :rarm :angle-vector (cdr arm-av))))))
       (cond ((memq arm '(:larm :rarm))
              (setq ret (send robot arm :inverse-kinematics (midcoords cur c0 c1)
                              :rotation-axis rotation-axis :translation-axis translation-axis)))
             (t
              (setq ret
                    (and (send robot :larm :inverse-kinematics (midcoords cur (car c0) (car c1))
                               :rotation-axis rotation-axis :translation-axis translation-axis)
                         (send robot :rarm :inverse-kinematics (midcoords cur (cdr c0) (cdr c1))
                               :rotation-axis rotation-axis :translation-axis translation-axis)))))
       (when (and (null ret) revert-if-fail)
         (ros::ros-error ":angle-vector-with-constraint ik failed~%")
         (send robot :angle-vector pav)
         (return-from :angle-vector-with-constraint nil))
       (cond ((memq arm '(:larm :rarm))
              (setq arm-av (send robot arm :angle-vector)))
             (t
              (setq arm-av (cons (send robot :larm :angle-vector)
                                 (send robot :rarm :angle-vector)))))
       (push (send robot :angle-vector) avs)
       (push (/ tm (float div)) tms))
     (setq avs (reverse avs))
     (send-super :angle-vector-sequence avs tms) ;; avoid :check-continuous-joint-move-over-180
     )) ; :angle-vector-with-constraint
  ) ;; defmethod pr2-interface

;; pr2 switch controllers
(defmethod pr2-interface
  (:list-controllers
   ()
   "Returns list of available controllers with cons where cdr is t if the controller is running state, nil otherwise.
   "
   (let ((srv-name "/pr2_controller_manager/list_controllers")
         res)
     (unless (ros::wait-for-service srv-name 10)
       (ros::ros-error "service ~A is not advertised" srv-name)
       (return-from :list-controllers nil))
     (setq res (ros::service-call srv-name
                                  (instance pr2_mechanism_msgs::ListControllersRequest :init)))
     (mapcar #'cons (send res :controllers)
             (mapcar #'(lambda (s) (string= s "running")) (send res :state)))))
  (:switch-controller
   (stop start &key (force))
   "Switch controller
    Args:
        stop: controller name or list of controllers to stop
        start: controller name or list of controllers to start
        force: switch with strict policy if set to t, switch with best effort otherwise.
    Returns: t if successfully switched controllers, nil otherwise.
   "
   (when (send self :simulation-modep)
     (ros::ros-warn "switching controller is disabled on simulation mode.")
     (return-from :switch-controller t))
   (let ((srv-name "/pr2_controller_manager/switch_controller")
         (req (instance pr2_mechanism_msgs::SwitchControllerRequest :init))
         available res)
     (unless (ros::wait-for-service srv-name 10)
       (ros::ros-error "service ~A is not advertised" srv-name)
       (return-from :switch-controller nil))
     (setq available (mapcar #'car (send self :list-controllers)))
     (when (set-difference (flatten stop) available :test #'string=)
       (error "Unknown controller: ~A" (set-difference (flatten stop) available :test #'string=)))
     (when (set-difference (flatten start) available :test #'string=)
       (error "Unknown controller: ~A" (set-difference (flatten start) available :test #'string=)))
     (send req :stop_controllers (or (flatten stop) "dummy_controller"))
     (send req :start_controllers (or (flatten start) "dummy_controller"))
     (send req :strictness
           (if force
               pr2_mechanism_msgs::SwitchControllerRequest::*STRICT*
               pr2_mechanism_msgs::SwitchControllerRequest::*BEST_EFFORT*))
     (setq res (ros::service-call srv-name req))
     (if (send res :ok)
         (ros::ros-info "switched controller ~A -> ~A" stop start)
         (ros::ros-error "failed to switch controller ~A -> ~A" stop start))
     (send res :ok)))
  (:start-mannequin-mode
   (&optional (controller t))
   "Switch controller to loose_controller.
    Args:
        controller: :head :arms :rarm :larm, list of controllers or t for all controllers
    Returns: t if success, nil otherwise.
    "
   (when (eq controller t)
     (setq controller (list :head :arms)))
   (setq controller (flatten controller))
   (let (stop start)
     (when (memq :head controller)
       (push "head_traj_controller" stop)
       (push "head_traj_controller_loose" start))
     (when (or (memq :larm controller) (memq :arms controller))
       (push "l_arm_controller" stop)
       (push "l_arm_controller_loose" start))
     (when (or (memq :rarm controller) (memq :arms controller))
       (push "r_arm_controller" stop)
       (push "r_arm_controller_loose" start))
     (send self :switch-controller stop start)))
  (:stop-mannequin-mode
   (&optional (controller t))
   "Switch controller from loose_controller to normal controller.
    Args:
        controller: :head :arms :rarm :larm, list of controllers or t for all controllers
    Returns: t if success, nil otherwise.
    "
   (when (eq controller t)
     (setq controller (list :head :arms)))
   (setq controller (flatten controller))
   (let (stop start)
     (when (memq :head controller)
       (push "head_traj_controller_loose" stop)
       (push "head_traj_controller" start))
     (when (or (memq :larm controller) (memq :arms controller))
       (push "l_arm_controller_loose" stop)
       (push "l_arm_controller" start))
     (when (or (memq :rarm controller) (memq :arms controller))
       (push "r_arm_controller_loose" stop)
       (push "r_arm_controller" start))
     (send self :switch-controller stop start))))

;; pr2 costmap
(defmethod pr2-interface
  (:change-inflation-range
   (&optional (range 0.3))
   "Changes inflation range of local costmap for obstacle avoidance."
   (send-super :change-inflation-range range
               :node-name "move_base_node"
               :costmap-name "local_costmap"
               :inflation-name "inflation_layer"))
) ;; pr2 costmap

;;;;;
;;;;; utility functions pr2 robot
;;;;;
(defun pr2-init (&optional (create-viewer))
  ;; env
  (unless (boundp '*pr2*) (pr2))
  (unless (ros::ok) (ros::roseus "pr2_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)

  (when create-viewer (objects (list *pr2*)))
  )

;; (list larm-v rarm-v)
(defconstant *pr2-tuckarm-pose-rarm-free* (list #f(5 70 105 -90 -70 -6 -20)
                                                #f(0 60 -80 -121 -15 -90 20)))
(defconstant *pr2-tuckarm-pose-larm-free* (list #f(0 60 80 -121 15 -90 -20)
                                                #f(-5 70 -105 -90 70 -6 20)))
(defconstant *pr2-tuckarm-pose-rarm-free-outside* (list #f(4 74.3 110.3 -82.3 -80.2 -5.7 -20)
                                                        #f(0.4 74.3 -102.3 -121.5 78.6 -114.6 58.1)))
(defconstant *pr2-tuckarm-pose-larm-free-outside* (list #f(-0.4 74.3 102.3 -121.5 -78.6 -114.6 -58.1)
                                                        #f(-4 74.3 -110.3 -82.3 80.2 -5.7 20)))

(defun get-tuckarm (free-arm dir arm)
  (if (eq free-arm :larm)
      (if (eq dir :inside)
          (if (eq arm :larm)
              (car *pr2-tuckarm-pose-larm-free*)
            (cadr *pr2-tuckarm-pose-larm-free*)
            )
        (if (eq arm :larm)
            (car *pr2-tuckarm-pose-larm-free-outside*)
          (cadr *pr2-tuckarm-pose-larm-free-outside*)
          )
        )
    (if (eq dir :inside)
        (if (eq arm :larm)
            (car *pr2-tuckarm-pose-rarm-free*)
          (cadr *pr2-tuckarm-pose-rarm-free*)
          )
      (if (eq arm :larm)
          (car *pr2-tuckarm-pose-rarm-free-outside*)
        (cadr *pr2-tuckarm-pose-rarm-free-outside*)
        )
      )
    )
  )

(defun check-tuckarm-pose (&key (thre 20) &rest args)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((l-angle (map float-vector #'(lambda(d)(- d (* 360 (round (/ d 360.0)))))
                      (send *pr2* :larm :angle-vector)))
        (r-angle (map float-vector #'(lambda(d)(- d (* 360 (round (/ d 360.0)))))
                      (send *pr2* :rarm :angle-vector)))
	(weight #f(5 2 1 0.5 0.1 0 0))
        is-rarm is-larm)
    (setq is-larm (and (< (norm (map float-vector #'* (v- l-angle (car *pr2-tuckarm-pose-larm-free*)) weight)) thre)
                       (< (norm (map float-vector #'* (v- r-angle (cadr *pr2-tuckarm-pose-larm-free*)) weight)) thre)))
    (setq is-rarm (and (< (norm (map float-vector #'* (v- l-angle (car *pr2-tuckarm-pose-rarm-free*)) weight)) thre)
                       (< (norm (map float-vector #'* (v- r-angle (cadr *pr2-tuckarm-pose-rarm-free*)) weight)) thre)))
    (cond ((and (memq :rarm args) is-rarm) :rarm)
          ((and (memq :larm args) is-larm) :larm)
          (is-rarm :rarm)
          (is-larm :larm))
    ))

;; send pr2 to move to tuckarm pose if not the pose now
;; args is set the arm to move freely
(defun pr2-tuckarm-pose (&rest args)
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (free-arm (or (car args) current-arm :larm))
         (side (or (cadr args) :inside))
         (msec 500))
    (when (not (eq current-arm free-arm))
      (progn
        (setq msec 2000)
        (send *pr2* :larm :angle-vector #f( 25 0 0 -121 0 -6 0))
        (send *pr2* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        ))
    (if (eq free-arm :larm)
        (progn
          (send *pr2* :rarm :angle-vector (get-tuckarm :larm side :rarm))
          (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
        (send *pr2* :larm :angle-vector (get-tuckarm :rarm side :larm))
        (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec)))
    (send *ri* :wait-interpolation)
    (if (eq free-arm :larm)
        (progn
          (send *pr2* :larm :angle-vector (get-tuckarm :larm side :larm))
          (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
        (send *pr2* :rarm :angle-vector (get-tuckarm :rarm side :rarm))
        (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec)))
    (send *ri* :wait-interpolation)
    t
    ))

;; send pr2 to move to reset pose
(defun pr2-reset-pose ()
  (let ()
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ))

;; do not use tilt laser
(defun use-tilt-laser-obstacle-cloud (enable)
  ;; check if we have service call
  (let ((req (instance topic_tools::MuxSelectRequest :init
                       :topic (if enable "ground_object_cloud" "empty_cloud"))))
    (ros::service-call "/tilt_laser_mux/select" req)))
