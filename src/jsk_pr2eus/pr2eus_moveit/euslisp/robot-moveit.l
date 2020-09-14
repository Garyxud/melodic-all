(ros::load-ros-manifest "moveit_msgs")

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :collision-object-publisher "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(defvar *moveit-error-code-list*
  (list
   (cons 1 "SUCCESS")
   (cons 99999 "FAILURE")
   ;;
   (cons -1 "PLANNING_FAILED")
   (cons -2 "INVALID_MOTION_PLAN")
   (cons -3 "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE")
   (cons -4 "CONTROL_FAILED")
   (cons -5 "UNABLE_TO_AQUIRE_SENSOR_DATA")
   (cons -6 "TIMED_OUT")
   (cons -7 "PREEMPTED")
   ;; planning & kinematics request errors
   (cons -10 "START_STATE_IN_COLLISION")
   (cons -11 "START_STATE_VIOLATES_PATH_CONSTRAINTS")
   ;;
   (cons -12 "GOAL_IN_COLLISION")
   (cons -13 "GOAL_VIOLATES_PATH_CONSTRAINTS")
   (cons -14 "GOAL_CONSTRAINTS_VIOLATED")
   ;;
   (cons -15 "INVALID_GROUP_NAME")
   (cons -16 "INVALID_GOAL_CONSTRAINTS")
   (cons -17 "INVALID_ROBOT_STATE")
   (cons -18 "INVALID_LINK_NAME")
   (cons -19 "INVALID_OBJECT_NAME")
   ;; system errors
   (cons -21 "FRAME_TRANSFORM_FAILURE")
   (cons -22 "COLLISION_CHECKING_UNAVAILABLE")
   (cons -23 "ROBOT_STATE_STALE")
   (cons -24 "SENSOR_INFO_STALE")
   ;; kinematics errors
   (cons -31 "NO_IK_SOLUTION")
   ))

(defclass moveit-environment
  :super propertied-object
  :slots (config-list
          scene-service
          planning-service
          execute-service
          check-validity-service
          query-planner-interface-service
          planning-scene-world-topic
          robot
          default-frame-id default-link
          default-planner-id
          multi-dof-name multi-dof-frame
          use-action planning-action-client
          ))

;; frame-id
;; multi-dof-joint name/frame-id
;; group-name -> joint-list, target-link
(defmethod moveit-environment
  (:init
   (&key ((:scene-service sc-srv) "/get_planning_scene")
         ((:planning-service pl-srv) "/plan_kinematic_path")
         ((:execute-service ex-srv) "/execute_kinematic_path")
         ((:query-planner-interface-service qr-pl-srv) "/query_planner_interface")
         ((:planning-scene-world pl-sc-world) "/planning_scene_world")
         ((:state-validity-service sv-srv) "/check_state_validity")
         ((:robot rb)) (frame-id) ;; frame-id needs to be contained in robot_model
         ((:planner-id pl-id) "RRTConnectkConfigDefault")
         (multi-dof-joint-name "virtual_joint")
         (multi-dof-frame-id "odom_combined"))
   (unless rb
     (warn "subclass's respoinsibility to specify :robot in (send ~s :init)~%" self)
     (return-from :init nil))
   (unless frame-id
     (warn "subclass's respoinsibility to specify :frame-id in (send ~s :init)~%" self)
     (return-from :init nil))
   (setq scene-service sc-srv
         planning-service pl-srv
         execute-service ex-srv
         check-validity-service sv-srv
         query-planner-interface-service qr-pl-srv
         planning-scene-world-topic pl-sc-world
         robot rb
         default-frame-id frame-id
         default-planner-id pl-id
         multi-dof-name multi-dof-joint-name
         multi-dof-frame multi-dof-frame-id)
   (ros::advertise planning-scene-world-topic moveit_msgs::PlanningSceneWorld)
   (setq default-link (send self :search-link-from-name frame-id))
   (setq config-list (send self :default-configuration))
   (unless (ros::ok)
     (ros::roseus "robot_moveit_environment"))
   self)
  (:multi-dof-joint-name (&optional jt)
   (if jt (setq multi-dof-name jt)) multi-dof-name)
  (:multi-dof-joint-frame-id (&optional frame)
   (if frame (setq multi-dof-frame frame)) multi-dof-frame)
  (:default-frame-id (&optional frame)
   (if frame (setq default-frame-id frame) default-frame-id))
  (:robot (&rest args) (forward-message-to robot args))
  (:sync-robot-model (rb &optional (change-argument nil))
   (let ((r-from (if change-argument robot rb))
         (r-to   (if change-argument rb robot)))
     (send r-to :reset-coords)
     (send r-to :transform (send r-from :worldcoords))
     (send r-to :angle-vector (send r-from :angle-vector))
     (send r-to :worldcoords)
     r-to))
  (:init-action-client ()
   (unless planning-action-client
     (setq planning-action-client
           (instance ros::simple-action-client :init
                     "/move_group" moveit_msgs::MoveGroupAction
                     :groupname "robot_moveit"))
     (send planning-action-client :wait-for-server)
     ))
  (:use-action (&optional (use :use-action)) (unless (eq use :use-action) (setq use-action use)) use-action)
  (:action-client (&rest args) (forward-message-to planning-action-client args))
  (:search-link-from-name
   (link-name)
   (cond
    ((find-method robot (intern (string-upcase (format nil "~A_lk" link-name)) *keyword-package*))
     (send robot (intern (string-upcase (format nil "~A_lk" link-name)) *keyword-package*)))
    ((find-method robot (intern (string-upcase link-name) *keyword-package*))
     (send robot (intern (string-upcase link-name) *keyword-package*)))
    (t
     (find-if #'(lambda (l) (cond ((symbolp (send l :name)) (string= (symbol-string (send l :name)) link-name))
                                  ((stringp (send l :name)) (string= (send l :name) link-name))
                                  (t nil))) (send robot :links))
     )))
  (:copy-robot-state (rb)
   (send robot :reset-coords)
   (send robot :transform (send rb :worldcoords))
   (send robot :angle-vector (send rb :angle-vector))
   robot)
  (:default-configuration ()
   (warn "subclass's respoinsibility (send ~s :default-configuration)~%" self))
  (:worldcoords->default-frame-relative (wcds)
   (send (send default-link :worldcoords) :transformation
         (send wcds :worldcoords)))
  (:get-group-tip-coords (confkey) ;; get-robot-coords
   (let ((link-inst (cdr (assoc :target-link (cdr (assoc confkey config-list))))))
     (send link-inst :copy-worldcoords)))
  (:query-planner-interface ()
   (let ((req (instance moveit_msgs::QueryPlannerInterfacesRequest :init))
         ret)
    (setq ret (ros::service-call query-planner-interface-service req))
    (when ret
      (send ret :planner_interfaces))
    ))
  (:get-planning-scene (&optional (components 1023))
   (get-planning-scene :scene-service scene-service :components components))
  (:clear-world-scene ()
   (let ((msg (instance moveit_msgs::PlanningSceneWorld :init)))
     (ros::publish planning-scene-world-topic msg)))
  (:check-state-validity ()
   (let* ((rmsg (send self :get-planning-scene moveit_msgs::PlanningSceneComponents::*ROBOT_STATE*))
          (msg (instance moveit_msgs::GetStateValidityRequest :init :robot_state (send rmsg :robot_state))))
     (ros::service-call check-validity-service msg)
     ))
  (:convert-end-coords
   (cds confkey end-coords )
   (let ((tgt-cds (send cds :copy-worldcoords))
         (rcds (send self :get-group-tip-coords confkey)))
     (send tgt-cds :transform
           (send (send (send* robot end-coords) :worldcoords)
                 :transformation rcds))
     tgt-cds
     ))
  (:get-ik-for-pose
   (cds confkey &rest args &key (use-actual-seed t) (retry t)
        (end-coords) ;; (list :rarm :end-coords)
        (frame-id default-frame-id) (timeout 0.05) (scene)
        (attempts) (avoid-collision t) (use-multi-dof nil) &allow-other-keys)
   (let ((tgt-cds (if end-coords
                      (send self :convert-end-coords cds confkey end-coords)
                    (send cds :copy-worldcoords)))
         (group-name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
         (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
         scene rstate constraints)
     ;;
     (when use-actual-seed
       (unless scene
         (setq scene (send self :get-planning-scene)))
       (when scene (setq rstate (send scene :robot_state))))
     ;;
     (setq tgt-cds (send self :worldcoords->default-frame-relative tgt-cds))
     ;;
     (let* ((msg (ros::coords->tf-pose-stamped tgt-cds frame-id))
            (req (instance moveit_msgs::GetPositionIKRequest :init
                           :ik_request
                           (instance moveit_msgs::PositionIKRequest :init
                                     :group_name group-name
                                     :avoid_collisions avoid-collision
                                     :constraints (if constraints constraints
                                                    (instance moveit_msgs::constraints :init))
                                     :robot_state (if rstate rstate
                                                    (instance moveit_msgs::RobotState :init))
                                     :attempts (if attempts attempts 0)
                                     :timeout (ros::time timeout)
                                     :pose_stamped msg)))
            (res (ros::service-call "/compute_ik" req)))
       (when (and retry (/= (send res :error_code :val) 1))
         (send req :ik_request :attempts (if attempts (* 2 attempts) 2))
         (send req :ik_request :timeout (ros::time (* 2 timeout)))
         (setq res (ros::service-call "/compute_ik" req)))
       (cond
        ((= (send res :error_code :val) 1) ;; success
         (apply-joint_state (send res :solution :joint_state) robot)) ;; updates joint-list
        (t
         (warn ";; ik error at ~A~%"
               (assoc (send res :error_code :val) *moveit-error-code-list*))
         (return-from :get-ik-for-pose nil)))
       (if use-multi-dof
           (cons (send robot :angle-vector) (send res :solution :multi_dof_joint_state))
           (send robot :angle-vector))
       )))
  (:execute-trajectory
   (msg &key (wait nil))
   (let ((req (instance moveit_msgs::ExecuteKnownTrajectoryRequest :init
                        :trajectory msg)))
     (send req :wait_for_execution wait)
     (ros::service-call execute-service req)
     ;; parse req
     ))
  (:motion-plan ;; motion-plan for joint-constraints
   (confkey &rest args
            &key (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
            (tolerance-below 0.001) (tolerance-above 0.001)
            &allow-other-keys)
   (let ((const
          (instance moveit_msgs::constraints :init :name ""
                    :joint_constraints
                    (mapcar #'(lambda (jn)
                                (instance moveit_msgs::jointconstraint :init
                                          :joint_name (send jn :name)
                                          :position (send jn :ros-joint-angle)
                                          :tolerance_above tolerance-above
                                          :tolerance_below tolerance-below
                                          :weight 1.0))
                            joint-list))))
     (send* self :motion-plan-constraints confkey :goal-constraints (list const) args)
     ))
  (:motion-plan-constraints
   (confkey &rest args &key (scene) (planner-id default-planner-id)
            (planning-attempts 1) (planning-time 5.0)
            (workspace-x 1.0) (workspace-y 1.0) (workspace-z 1.6)
            (goal-constraints) (extra-goal-constraints)
            (path-constraints) (trajectory-constraints)
            (max-velocity-scale) (max-acceleration-scale)
            &allow-other-keys)
   (let ((group-name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
         (mpr (instance moveit_msgs::motionplanrequest :init))
         res)
     ;;
     (unless scene
       (setq scene (send self :get-planning-scene)))

     (send mpr :workspace_parameters :header :stamp (ros::time-now))
     (send mpr :workspace_parameters :header :frame_id multi-dof-frame);;
     ;;
     (send mpr :workspace_parameters :max_corner :x workspace-x)
     (send mpr :workspace_parameters :max_corner :y workspace-y)
     (send mpr :workspace_parameters :max_corner :z workspace-z)
     (send mpr :workspace_parameters :min_corner :x (- workspace-x))
     (send mpr :workspace_parameters :min_corner :y (- workspace-y))
     (send mpr :workspace_parameters :min_corner :z (- workspace-z))
     ;;
     (send mpr :start_state (send scene :robot_state))

     (when goal-constraints
       (if (atom goal-constraints) (setq goal-constraints (list goal-constraints)))
       (send mpr :goal_constraints goal-constraints))
     (when extra-goal-constraints
       (if (atom extra-goal-constraints)
           (setq extra-goal-constraints (list extra-goal-constraints)))
       (nconc (send mpr :goal_constraints) extra-goal-constraints)
       (send mpr :goal_constraints
             (list (merge-goal-constraints (send mpr :goal_constraints)))))
     (if path-constraints (send mpr :path_constraints path-constraints))
     ;; TrajectoryConstraints is not used in plannning
     ;; Detailed info: https://github.com/ros-planning/moveit_msgs/issues/2
     (if trajectory-constraints (send mpr :trajectory_constraints trajectory-constraints))
     (send mpr :planner_id planner-id) ;; select from :query-planner-interface
     (send mpr :group_name group-name)
     (send mpr :num_planning_attempts planning-attempts)
     (send mpr :allowed_planning_time planning-time)
     (when max-velocity-scale (send mpr :max_velocity_scaling_factor max-velocity-scale))
     (when max-acceleration-scale (send mpr :max_acceleration_scaling_factor max-acceleration-scale))
     ;;(print-ros-msg mpr)
     (when use-action
       (send self :init-action-client)
       (let ((goal (send planning-action-client :make-goal-instance)))
         (send goal :header :seq 1)
         (send goal :header :stamp (ros::time-now))
         (send goal :goal :request mpr)
         (send goal :goal :planning_options :plan_only t)
         (send planning-action-client :send-goal goal))
       (return-from :motion-plan-constraints nil))
     (setq res
           (ros::service-call planning-service
                              (instance moveit_msgs::GetMotionPlanRequest
                                        :init :motion_plan_request mpr)))
     (cond
      ((= (send res :motion_plan_response :error_code :val) 1) ;; success
       ;; have to do apply multi-dof-joint ...
       (send res :motion_plan_response))
      (t
       (warn ";; motion plan error at ~A~%"
             (assoc (send res :motion_plan_response :error_code :val)
                    *moveit-error-code-list*))
       (return-from :motion-plan-constraints nil)))
     ))
  (:move-arm-to-goal (confkey &rest args &key (wait) &allow-other-keys)
   (let ((ret (send* self :motion-plan args)))
     (when ret
       (send self :execute-trajectory (send ret :trajectory) :wait wait))))
  (:planning-make-trajectory
   (confkey &rest args &key (set-angle-vector) (scene) (use-scene t) (planning-time 5.0)
            (planning-attempts 3) (retry) &allow-other-keys)
   "Request trajectory plan from constraints"
   (let (ret)
     (when set-angle-vector (send robot :angle-vector set-angle-vector))
     (unless scene (setq scene (send self :get-planning-scene)))
     (setq ret
           (send* self :motion-plan confkey
                  :planning-time planning-time :planning-attempts planning-attempts
                  :scene (if use-scene scene)
                  args))
     (when (and retry (not ret)) ;; retry
       (when get-scene (setq scene (send self :get-planning-scene)))
       (setq ret
             (send* self :motion-plan confkey
                    :planning-time (* 2 planning-time)
                    :planning-attempts (* 2 planning-attempts)
                    :scene (if use-scene scene)
                    args)))
     ret
     ))
  (:planning-make-trajectory-to-coords-no-ik
   (cds confkey &rest args &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (scene) (frame-id default-frame-id)
        (planning-attempts 3) (retry) (use-scene t)
        ((:tolerance_x tx) 0) ((:tolerance_y ty) 0) ((:tolerance_z tz) 0)
        ((:tolerance_rx rx) 0) ((:tolerance_ry ry) 0) ((:tolerance_rz rz) 0)
        &allow-other-keys)
   (let ((tgt-cds (if end-coords
                      (send self :convert-end-coords cds confkey end-coords)
                    (send cds :copy-worldcoords)))
         const ret)
     (unless scene (setq scene (send self :get-planning-scene)))
     (setq tgt-cds (send self :worldcoords->default-frame-relative tgt-cds))
     (send (cdr (assoc :target-link (cdr (assoc confkey config-list)))) :name)
     (setq const
           (make-pose-constraints (send (cdr (assoc :target-link
                                                    (cdr (assoc confkey config-list)))) :name)
                                  tgt-cds :frame_id default-frame-id
                                  :tolerance_x tx :tolerance_y ty :tolerance_z tz
                                  :tolerance_rx rx :tolerance_ry ry :tolerance_rz rz))
     (setq ret
           (send* self :motion-plan-constraints confkey
                  :goal-constraints const
                  :planning-time planning-time :planning-attempts planning-attempts
                  :scene (if use-scene scene) args))
     (when (and retry (not ret)) ;; retry
       (when get-scene (setq scene (send self :get-planning-scene)))
       (setq ret
             (send* self :motion-plan-constraints confkey
                    :goal-constraints const
                    :planning-time (* 2 planning-time)
                    :planning-attempts (* 2 planning-attempts)
                    :scene (if use-scene scene) args)))
     ret
     ))
  (:multi-dof-joint-state->joint-constraints
   (js &optional joint-name)
   (unless joint-name (setq joint-name (send js :joint_names)))
   (if (atom joint-name) (setq joint-name (list joint-name)))
   (let (consts pos trans)
     (labels ((transform->coords (tr)
                (make-coords :pos (ros::tf-point->pos (send tr :translation))
                             :rot (ros::tf-quaternion->rot (send tr :rotation)))))
       (dolist (jn joint-name)
         (setq pos (position jn (send js :joint_names) :test #'string=))
         (when pos
           (setq trans (elt (send js :transforms) pos))
           (push (make-virtual-joint-constraints
                  (transform->coords (elt (send js :transforms) pos))
                  :joint-name jn) consts))))
     consts))
  (:planning-make-trajectory-to-coords ;; use-ik
   (cds confkey &rest args &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (scene) (frame-id default-frame-id)
        (planning-attempts 3) (retry) (use-scene t)
        (multi-dof-joint-constraints)
        &allow-other-keys)
   "Solve inverse-kinematics for constructing joint constraints and request a plan"
   (let (ret consts)
     (unless scene (setq scene (send self :get-planning-scene)))
     (setq ret (send self :get-ik-for-pose cds confkey :end-coords end-coords
                                                       :use-actual-seed t :retry retry :frame-id frame-id :scene scene :use-multi-dof t)) ;; (av . multi-dof-js) or nil
     (unless ret (return-from :planning-make-trajectory-to-coords nil))
     (when multi-dof-joint-constraints
       (if (member :extra-goal-constraints args)
           (nconc (cdr (assoc :extra-goal-constraints args))
                  (send self :multi-dof-joint-state->joint-constraints
                        (cdr ret) multi-dof-joint-constraints))
           (setq args (append args (list :extra-goal-constraints
                                         (send self :multi-dof-joint-state->joint-constraints
                                               (cdr ret) multi-dof-joint-constraints))))))
     (send* self :planning-make-trajectory confkey
           :planning-time planning-time :planning-attempts planning-attempts
           :use-scene use-scene :scene scene :retry retry args)
     ))
  (:planning-move-arm
   (confkey &key (set-angle-vector) (scene) (use-scene t) (planning-time 5.0)
            (planning-attempts 3) (retry) (wait t) &allow-other-keys)
   (let (ret)
     (if set-angle-vector (send robot :angle-vector set-angle-vector))
     (unless scene (setq scene (send self :get-planning-scene)))
     (setq ret
           (send self :move-arm-to-goal confkey :scene (if use-scene scene)
                 :planning-attempts planning-attempts
                 :planning-time planning-time :wait wait))
     (when (and retry (not ret)) ;; retry
       (if scene (setq scene (send self :get-planning-scene))) ;; override scene
       (setq ret
             (send self :move-arm-to-goal confkey :scene (if use-scene scene)
                   :planning-attempts (* 2 planning-attempts)
                   :planning-time (* 2 planning-time) :wait wait))
       )
     ret
     ))
  (:planning-move-arm-to-coords
   (cds confkey &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (scene) (frame-id default-frame-id)
        (planning-attempts 3) (retry) (use-scene t) (wait t)
        &allow-other-keys)
   (let (ret)
     (unless scene (setq scene (send self :get-planning-scene)))
     (unless (send self :get-ik-for-pose cds confkey :end-coords end-coords
                   :use-actual-seed t :retry retry :frame-id frame-id :scene scene)
       (return-from :planning-move-arm-to-coords nil))
     (send self :planning-move-arm confkey
           :planning-time planning-time :planning-attempts planning-attempts
           :use-scene use-scene :wait wait :scene scene)
     ))
  )

(defun worldcoords->link-relative (wcds &key ((:link lname) "ROOT") (robot *pr2*))
  (let ((base (send robot (intern (string-upcase lname) *keyword-package*))))
    (send (send base :worldcoords) :transformation
          (send wcds :worldcoords))))

(defmethod robot-interface
  (:set-moveit-environment (&optional mv-env)
   (when mv-env (setf (get self :moveit-environment) mv-env))
   (get self :moveit-environment))
  (:planning-environment (&rest args)
   (let ((env (get self :moveit-environment)))
     (when env (forward-message-to env args))))
  (:update-planning-robot ()
   (send self :state)
   (send self :planning-environment
         :sync-robot-model robot))
  (:parse-end-coords (&rest args &key (move-arm) (use-torso) &allow-other-keys)
   (unless move-arm
     (error "keyword argument :move-arm must not be nil"))
   (let (confkey ed-lst)
     (cond
      ((eq move-arm :rarm)
       (setq confkey (if use-torso :rarm-torso :rarm))
       (setq  ed-lst (list :rarm :end-coords)))
      ((eq move-arm :arms)
       (setq confkey (if use-torso :arms-torso :arms))
       (setq ed-lst nil)) ;; can not use inverse-kinematics
      (t ;;(eq arm :larm)
       (setq confkey (if use-torso :larm-torso :larm))
       (setq  ed-lst (list :larm :end-coords))))
     (cons confkey ed-lst)))
  (:collision-aware-ik
   (cds &rest args &key (move-arm :larm) (use-torso) &allow-other-keys)
   (let* ((r (send* self :parse-end-coords args))
          (confkey (car r))
          (ed-lst (cdr r))
          ret)
     (setq ret
           (send* self :planning-environment
                  :get-ik-for-pose cds confkey :end-coords ed-lst args))
     ret))
  (:angle-vector-make-trajectory
   (av &rest args)
   (let* ((r (send* self :parse-end-coords args))
          (confkey (car r))
          (ed-lst (cdr r))
          time-from-start
          scene points mdof-points
          tmp-ret ret)
     (when (atom av)
       (return-from :angle-vector-make-trajectory
         (send* self :planning-environment
                :planning-make-trajectory confkey
                :set-angle-vector av :end-coords ed-lst args)))
     (dolist (tmp-av av)
       (setq scene (send self :planning-environment :get-planning-scene))
       (send scene :robot_state :joint_state
             (joint-list->joint_state (send robot :joint-list)))
       (setq tmp-ret
             (send* self :planning-environment
                    :planning-make-trajectory confkey
                    :set-angle-vector tmp-av :scene scene
                    :end-coords ed-lst args))
       (unless tmp-ret
         (return-from :angle-vector-make-trajectory nil))
       (setq points (send tmp-ret :trajectory :joint_trajectory :points))
       (cond
         ((null ret)
          (setq ret tmp-ret))
         (t
          (setq mdof-points (send tmp-ret :trajectory :multi_dof_joint_trajectory :points))
          (when time-from-start
            (dolist (pt points)
              (send pt :time_from_start
                    (ros::time+ time-from-start (send pt :time_from_start))))
            (dolist (pt mdof-points)
              (send pt :time_from_start
                    (ros::time+ time-from-start (send pt :time_from_start)))))
          (send ret :planning_time (+ (send ret :planning_time) (send tmp-ret :planning_time)))
          (if (send ret :trajectory :joint_trajectory :points)
              (when points
                (when (ros::time= (send (car points) :time_from_start) time-from-start)
                  ;; First point of newly planned traj overlaps with last point of existing traj
                  (pop points))
                (nconc (send ret :trajectory :joint_trajectory :points) points))
              (send ret :trajectory :joint_trajectory :points points))
          (if (send ret :trajectory :multi_dof_joint_trajectory :points)
              (when mdof-points
                (when (ros::time= (send (car mdof-points) :time_from_start) time-from-start)
                  ;; First point of newly planned traj overlaps with last point of existing traj
                  (pop mdof-points))
                (nconc (send ret :trajectory :multi_dof_joint_trajectory :points) mdof-points))
              (send ret :trajectory :multi_dof_joint_trajectory :points mdof-points))))
       (setq time-from-start
             (copy-object (send (car (last points)) :time_from_start)))
       (send robot :angle-vector tmp-av))
     ret))
  (:end-coords-make-trajectory (cds &rest args)
   (let* ((r (send* self :parse-end-coords args))
          (confkey (car r))
          (ed-lst (cdr r)))
     (send* self :planning-environment
            :planning-make-trajectory-to-coords
            cds confkey :end-coords ed-lst args)))
  (:angle-vector-motion-plan ;;
   (av &rest args &key (ctype controller-type) (start-offset-time 0) (total-time)
       (move-arm :larm) (reset-total-time 5000.0) (use-send-angle-vector) (scale 1.0)
       &allow-other-keys)
   (let (traj ret orig-total-time sent-controllers other-joints)
     (setq ctype (or ctype controller-type))
     (unless (gethash ctype controller-table)
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector-motion-plan))
     (setq ret (send* self :angle-vector-make-trajectory av args))
     (unless ret (return-from :angle-vector-motion-plan nil))
     (setq traj (send ret :trajectory))
     (setq orig-total-time (send (send (car (last (send traj :joint_trajectory :points))) :time_from_start) :to-sec)) ;; [sec]
     (when (< orig-total-time 0.001)
       (unless reset-total-time
         (ros::ros-error "Trajectory has very short duration")
         (return-from :angle-vector-motion-plan nil))
       (ros::ros-warn "reset Trajectory Total time")
       (setq traj (send* self :trajectory-filter traj :total-time reset-total-time args)))
     (ros::ros-info ";; Planned Trajectory Total Time ~7,3f [sec]" orig-total-time)
     (setq total-time
           (cond
             ((eq total-time :fast) (* scale (* orig-total-time 1000)))
             ((or (null total-time) (> orig-total-time (/ total-time 1000.0))) (* orig-total-time 1000))
             (t total-time)))
     (setq traj (send* self :trajectory-filter traj :total-time total-time args))
     (ros::ros-info ";; Scaled Trajectory Total Time ~0,3f(~0,3f) [sec]" (send (send (car (last (send traj :joint_trajectory :points))) :time_from_start) :to-sec) (/ total-time 1000.0))
     (ros::ros-info ";; generated ~A points for ~A sec using [~A] group" (length (send traj :joint_trajectory :points)) (/ total-time 1000.0) (send ret :group_name))
     (ros::ros-info ";; will send to ~A" (send traj :joint_trajectory :joint_names))
     ;;
     (mapcar
      #'(lambda (action param)
          ;; action : method (:larm-controller)
          ;; param  : definitions ((controller-action, controller-state, action-type, joint-names))
          ;;
          ;; If planned trajectory contains the joints that is not included in move_group
          ;; search controller table and send the trajectory to the controller
          (setq other-joints (not (intersection (send traj :joint_trajectory :joint_names) (cdr (assoc :joint-names param)) :test #'string=)))
          (when other-joints
            (maphash #'(lambda (ac ct)
                         (when (and (= (length ct) 1)
                                    (equal (send-all (list action) :name) (send-all ct :name))
                                    (not (member (send (car ct) :name) sent-controllers :test #'string=)))
                           (ros::ros-info ";; send self :angle-vector ~A ~A (without planning)" ac ct)
                           (push (send (car ct) :name) sent-controllers)
                           (if (listp av)
                               (send-message self robot-interface :angle-vector-sequence av total-time ac)
                               (send-message self robot-interface :angle-vector av total-time ac))))
                     controller-table)))
      (gethash ctype controller-table) (send self ctype))
     (cond
       (use-send-angle-vector
        (send* self :joint-trajectory-to-angle-vector-list
               move-arm (send traj :joint_trajectory) args))
       (t
        (ros::ros-info ";; send self :send-trajectory ~A" ctype)
        (if start-offset-time
            (send* self :send-trajectory (send traj :joint_trajectory)
                        :controller-type ctype :starttime start-offset-time args)
            (send* self :send-trajectory (send traj :joint_trajectory)
                        :controller-type ctype args))))
     ret))
  (:move-end-coords-plan ;;
   (cds &rest args &key (move-arm :larm) (reset-total-time 5000.0) (use-send-angle-vector) &allow-other-keys)
   (let (traj ret orig-total-time)
     (setq ret (send* self :end-coords-make-trajectory cds args))
     (unless ret (return-from :move-end-coords-plan nil))
     (setq traj (send ret :trajectory))
     (setq traj (send* self :trajectory-filter traj :total-time reset-total-time args))
     (setq orig-total-time (send (send (car (last (send traj :joint_trajectory :points))) :time_from_start) :to-sec))
     (when (< orig-total-time 0.001)
       (ros::ros-error "Trajectory has very short duration")
       (return-from :move-end-coords-plan nil))
     (if use-send-angle-vector
         (send* self :joint-trajectory-to-angle-vector-list
                move-arm (send traj :joint_trajectory) args)
         (send* self :send-trajectory (send traj :joint_trajectory) args))
     ret))
  (:trajectory-filter ;; simple trajectory for zero duration
   (traj &key (copy) (total-time 5000.0) (clear-velocities) &rest args &allow-other-keys)
   "traj (moveit_msgs/RobotTrajectory): input trajectory"
   (let ((orig-total-time (send (send (car (last (send traj :joint_trajectory :points))) :time_from_start) :to-sec)))
     ;; check if valid filtering can be applied
     (when (null total-time)
       (ros::ros-debug ";; Trajectory filter is skipped")
       (return-from :trajectory-filter traj))

     (when copy
       (setq traj (copy-object traj)))
     (let* ((points (send traj :joint_trajectory :points))
            (mdof-points (send traj :multi_dof_joint_trajectory :points))
            (size (length points))
            (time-step (/ 1 (float (1- size))))
            (time-scale (/ (/ total-time 1000.0) orig-total-time)))
       (dolist (pt points)
         (if clear-velocities
           (progn
             (send pt :velocities #f())
             (send pt :accelerations #f()))
           (progn
             (send pt :velocities
                   (map float-vector #'(lambda (x) (/ x time-scale)) (send pt :velocities)))
             (send pt :accelerations
                   (map float-vector #'(lambda (x) (/ x (expt time-scale 2))) (send pt :accelerations)))))
         (send pt :time_from_start (ros::time (* time-scale (send (send pt :time_from_start) :to-sec)))))
       (dolist (pt mdof-points)
         (if clear-velocities
           (progn
             (send pt :velocities nil)
             (send pt :accelerations nil))
           (progn
             (dolist (twist-key (list :linear :angular))
               (dolist (vec-key (list :x :y :z))
                 (dolist (vel (send pt :velocities))
                   (send vel twist-key vec-key
                         (/ (send vel twist-key vec-key) time-scale)))
                 (dolist (acc (send pt :accelerations))
                   (send acc twist-key vec-key
                         (/ (send acc twist-key vec-key) (expt time-scale 2))))))))
         (send pt :time_from_start (ros::time (* time-scale (send (send pt :time_from_start) :to-sec))))))
       traj))
  ) ;; robot-interface

(defmethod robot-move-base-interface
  (:parse-end-coords
   (&rest args &key (move-arm) (use-torso) (use-base) &allow-other-keys)
   (let (confkey ed-lst)
     (setq ed-lst (cdr (send-super* :parse-end-coords args)))
     (setq confkey (string move-arm))
     (when use-torso (setq confkey (format nil "~A-torso" confkey)))
     (when use-base (setq confkey (format nil "~A-base" confkey)))
     (cons (intern (string-upcase confkey) *keyword-package*) ed-lst)))
  (:collision-aware-ik
   (cds &rest args &key (use-base) &allow-other-keys)
   (let* ((r (send* self :parse-end-coords args))
          (confkey (car r))
          (ed-lst (cdr r)))
     (send* self :planning-environment
            :get-ik-for-pose cds confkey :end-coords ed-lst :use-multi-dof use-base args)))
  (:angle-vector-make-trajectory
   (av &rest args)
   (when (member :use-base args)
     ;; set current base position to goal constraints
     (let* ((odom->base (send *tfl* :lookup-transform
                              (send self :planning-environment :default-frame-id)
                              (send self :planning-environment :multi-dof-joint-frame-id) (ros::time 0)))
            (consts (list (make-virtual-joint-constraints
                           odom->base :joint-name (send self :planning-environment :multi-dof-joint-name)))))
         (if (member :extra-goal-constraints args)
             (nconc (cdr (assoc :extra-goal-constraints args)) consts)
             (setq args (append args (list :extra-goal-constraints consts))))))
   (send-super* :angle-vector-make-trajectory av args))
  (:end-coords-make-trajectory
   (cds &rest args)
   ;; TODO set base position as initial state
   (send-super* :end-coords-make-trajectory cds
                :multi-dof-joint-constraints (send self :planning-environment :multi-dof-joint-name)  args))
  (:execute-base-trajecotry
   (traj &key (send-action))
   (let ((base-pos (position (send self :planning-environment :multi-dof-joint-name)
                             (send traj :joint_names) :test #'string=))
         (base->odom (send *tfl* :lookup-transform
                           (send self :planning-environment :default-frame-id)
                           (send self :planning-environment :multi-dof-joint-frame-id)
                           (ros::time 0)))
         (prev-time 0.0) trans times)
     (labels ((transform->coords (tr)
                (make-coords :pos (ros::tf-point->pos (send tr :translation))
                             :rot (ros::tf-quaternion->rot (send tr :rotation)))))
       (dolist (pts (send traj :points))
         (push (transform->coords (elt (send pts :transforms) base-pos)) trans)
         (push (* 1000 (- (send (send pts :time_from_start) :to-sec) prev-time)) times)
         (setq prev-time (send (send pts :time_from_start) :to-sec))))
     (setq times (cdr (nreverse times)))
     (setq trans (mapcar #'(lambda (cds)
                             (send (send base->odom :copy-worldcoords)
                                   :transform cds)) (cdr (nreverse trans))))
     (setq trans (mapcar #'(lambda (cds)
                             (float-vector
                              (/ (elt (send cds :worldpos) 0) 1000.0)
                              (/ (elt (send cds :worldpos) 1) 1000.0)
                              (caar (send cds :rpy-angle)))) trans))
     (send self :move-trajectory-sequence
           trans times :send-action send-action)))
  (:angle-vector-motion-plan ;;
   (av &rest args &key (ctype controller-type) (start-offset-time 0) (total-time) (use-base)
       (move-arm :larm) (reset-total-time 5000.0) (use-send-angle-vector) (scale 1.0)
       &allow-other-keys)
   (let ((ret (send-super* :angle-vector-motion-plan av
                           :ctype ctype
                           :start-offset-time start-offset-time
                           :total-time total-time
                           :move-arm move-arm
                           :reset-total-time reset-total-time
                           :use-send-angle-vector use-send-angle-vector
                           :scale scale
                           args)))
     (when (and ret use-base)
       (send self :execute-base-trajecotry (send ret :trajectory :multi_dof_joint_trajectory)
                  :send-action t))
     ret))
  (:move-end-coords-plan
   (cds &rest args &key (move-arm :larm) (reset-total-time 5000.0) (use-send-angle-vector) (use-base) &allow-other-keys)
   (let ((ret (send-super* :move-end-coords-plan cds
                           :move-arm move-arm
                           :reset-total-time reset-total-time
                           :use-send-angle-vector use-send-angle-vector
                           args)))
     (when (and ret use-base)
       (send self :execute-base-trajecotry (send ret :trajectory :multi_dof_joint_trajectory)
                  :send-action t))
     ret))
  ) ;; robot-move-base-interface


(defun make-box-shape (x &optional y z)
  (let ((dim (float-vector x (if y y x) (if z z x))))
    (scale 0.001 dim dim)
    (instance shape_msgs::SolidPrimitive :init :type shape_msgs::SolidPrimitive::*BOX*
              :dimensions dim)))
(defun make-sphere-shape (r)
  (let ((dim (float-vector (* 0.001 r))))
    (instance shape_msgs::SolidPrimitive :init :type shape_msgs::SolidPrimitive::*SPHERE*
              :dimensions dim)))

(defun make-bounding-volume (pose type x &optional y z)
  (case type
    (:box
     (instance moveit_msgs::BoundingVolume :init
               :primitives (list (make-box-shape x y z))
               :primitive_poses (list (if pose pose (ros::identity-pose)))))
    (:sphere
     (instance moveit_msgs::BoundingVolume :init
               :primitives (list (make-sphere-shape x))
               :primitive_poses (list (if pose pose (ros::identity-pose)))))
    ))

(defun make-position-constraints (link_name target-coords &key (weight 1.0) (frame_id "world")
                                            (ignore-volume-orientation)
                                            (offset-position) (tolerance_r) (shape) (volume)
                                            (tolerance_x 5) (tolerance_y 5) (tolerance_z 5)
                                            &allow-other-keys)
  (let ((pose (ros::coords->tf-pose target-coords)))
    (if ignore-volume-orientation (send pose :orientation (ros::identity-quaternion)))
    (instance moveit_msgs::PositionConstraint :init
              :header (instance std_msgs::header :init :frame_id frame_id)
              :link_name link_name
              :target_point_offset
              (let ((g (instance geometry_msgs::Vector3 :init)))
                (when offset-position
                  (send g :x (* 0.001 (elt offset-position 0)))
                  (send g :y (* 0.001 (elt offset-position 0)))
                  (send g :z (* 0.001 (elt offset-position 0))))
                g)
              :constraint_region
              (cond
               (volume volume)
               (shape
                (instance moveit_msgs::BoundingVolume :init
                          :primitives (list shape)
                          :primitive_poses (list pose)))
               (tolerance_r
                (make-bounding-volume pose :sphere tolerance_r))
               (t
                (make-bounding-volume pose :box tolerance_x tolerance_y tolerance_z)))
              :weight weight)))

(defun make-orientation-constraints (link_name target-coords &key (weight 1.0) (frame_id "world")
                                               (tolerance_rx 0.015) (tolerance_ry 0.015) (tolerance_rz 0.015)
                                               &allow-other-keys)
  (instance moveit_msgs::OrientationConstraint :init
            :header (instance std_msgs::header :init :frame_id frame_id)
            :link_name link_name
            :orientation (ros::rot->tf-quaternion (send target-coords :worldrot))
            :absolute_x_axis_tolerance tolerance_rx
            :absolute_y_axis_tolerance tolerance_ry
            :absolute_z_axis_tolerance tolerance_rz
            :weight weight))

(defun make-pose-constraints (link_name target-coords
                                        &rest args &key (name "") (use-position t) (use-orientation t)
                                        &allow-other-keys)
  (instance moveit_msgs::Constraints :init :name name
            :position_constraints (if use-position
                                      (list (apply #'make-position-constraints
                                                   link_name target-coords args)))
            :orientation_constraints (if use-orientation
                                         (list (apply #'make-orientation-constraints
                                                      link_name target-coords args))))
  )

(defun make-virtual-joint-constraints
  (target-coords &key (joint-name "world_joint") (type :planner)
                 (tolerance-above-list (float-vector 0.01 0.01 0.01 0.01 0.01 0.01))
                 (tolerance-below-list (float-vector 0.01 0.01 0.01 0.01 0.01 0.01))
                 (weight-list (float-vector 1 1 1 1 1 1)))
  (let (constraints
        (max-vec (coerce tolerance-above-list float-vector))
        (min-vec (coerce tolerance-below-list float-vector))
        (weight-vec (coerce weight-list float-vector)))
    (push
     (instance moveit_msgs::jointconstraint :init
               :joint_name (format nil "~A/x" joint-name)
               :position (* 0.001 (elt (send target-coords :worldpos) 0))
               :tolerance_above (elt max-vec 0)
               :tolerance_below (elt min-vec 0)
               :weight (elt weight-vec 0))
     constraints)
    (push
     (instance moveit_msgs::jointconstraint :init
               :joint_name (format nil "~A/y" joint-name)
               :position (* 0.001 (elt (send target-coords :worldpos) 1))
               :tolerance_above (elt max-vec 1)
               :tolerance_below (elt min-vec 1)
               :weight (elt weight-vec 1))
     constraints)
    (push
     (instance moveit_msgs::jointconstraint :init
               :joint_name (format nil "~A/theta" joint-name)
               :position (* 0.001 (elt (send target-coords :worldpos) 1))
               :tolerance_above (elt max-vec 2)
               :tolerance_below (elt min-vec 2)
               :weight (elt weight-vec 2))
     constraints)
    (instance moveit_msgs::constraints :init :name ""
              :joint_constraints constraints)
    ))

(defun merge-goal-constraints (const-list)
  (instance moveit_msgs::Constraints :init
            :name (apply #'concatenate string (send-all const-list :name))
            :joint_constraints (flatten (send-all const-list :joint_constraints))
            :position_constraints (flatten (send-all const-list :position_constraints))
            :orientation_constraints  (flatten (send-all const-list :orientation_constraints))
            :visibility_constraints (flatten (send-all const-list :visibility_constraints))))


(provide :robot-moveit "robot-moveit.l")
