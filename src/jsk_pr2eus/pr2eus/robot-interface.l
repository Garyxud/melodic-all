;;;
;;; robot interface to ROS based pr2 system
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "rosgraph_msgs")
(ros::load-ros-manifest "control_msgs")
;;(ros::roseus-add-msgs "sensor_msgs") ;; roseus depends on sensor_msgs
;;(ros::roseus-add-msgs "visualization_msgs") ;; roseus depends on visualization_msgs
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "nav_msgs")
(require :speak "package://pr2eus/speak.l")

;; add ros-joint-angle method using meter/radian
(defmethod rotational-joint
  (:ros-joint-angle
   (&optional v &rest args)
   (if v (setq v (rad2deg v)))
   (setq v (send* self :joint-angle v args))
   (deg2rad v))
  )
(defmethod linear-joint
  (:ros-joint-angle
   (&optional v &rest args)
   (if v (setq v (* 1000.0 v)))
   (setq v (send* self :joint-angle v args))
   (* 0.001 v))
  )

(defclass controller-action-client
  :super ros::simple-action-client
  :slots (time-to-finish last-feedback-msg-stamp
          ri angle-vector-sequence timer-sequence current-time current-angle-vector previous-angle-vector scale-angle-vector;; slot for angle-vector-sequence
          ))
(defmethod controller-action-client
  (:init (r &rest args)
    (setq ri r) ;; robot-interface
    (setq time-to-finish 0)
    (setq last-feedback-msg-stamp (ros::time-now)) ;; this is for real robot
    (send-super* :init args))
  (:last-feedback-msg-stamp () last-feedback-msg-stamp)
  (:time-to-finish ()
    (ros::ros-debug "[~A] time-to-finish ~A" ros::name-space time-to-finish)
    time-to-finish)
  (:action-feedback-cb (msg)
   (let ((finish-time) (current-time))
     (ros::ros-debug "[~A] feedback-cb ~A" ros::name-space msg)
     (setq last-feedback-msg-stamp (send msg :header :stamp))
     (unless (and (send ros::comm-state :action-goal)
                  (not (equal (send (class ros::action-spec) :name)
                              'control_msgs::SingleJointPositionAction)))
       (return-from :action-feedback-cb nil))
     (cond ((derivedp msg control_msgs::followjointtrajectoryactionfeedback)
            (setq current-time (send msg :feedback :error :time_from_start)))
           (t
            (ros::ros-warn "feedback message type is not control_msgs/FollowJointTrajectoryActionFeedback.")
            ;; e.g. Type: pr2_controllers_msgs/JointTrajectoryActionFeedback does not have :error
            (setq current-time (ros::time- (ros::time-now) (send msg :status :goal_id :stamp)))))
     (setq finish-time (send (car (last (send (send ros::comm-state :action-goal) :goal :trajectory :points))) :time_from_start))
     (if (string= (send (send ros::comm-state :action-goal) :goal_id :id)
                  (send msg :status :goal_id :id))
         (setq time-to-finish (send (ros::time- finish-time current-time) :to-sec))
       (progn
         (ros::ros-warn "[~A] feedback-cb ~A received wrong goal" ros::name-space msg)
         (setq time-to-finish 0.0)))
     ))
  ;; method for simulation-modep
  (:push-angle-vector-simulation (av tm &optional prev-av)
   (setq previous-angle-vector prev-av)
   (setq angle-vector-sequence (append angle-vector-sequence (list av)))
   (setq timer-sequence (append timer-sequence (list tm)))
   (setq current-time 0)
   av)
  (:pop-angle-vector-simulation ()
    (let ((av (car angle-vector-sequence)) (tm (car timer-sequence)) scale-av)
      (when tm
        (setq scale-av (send ri :sub-angle-vector av previous-angle-vector))
        (setq av (v+ previous-angle-vector (scale (/ current-time tm) scale-av)))
        (incf current-time 100.0)
        (when (> current-time tm)
          (pop timer-sequence)
          (pop angle-vector-sequence)
          (setq av (v+ previous-angle-vector scale-av))
          (setq previous-angle-vector av)
          (setq current-time 0)
          )
        av)))
  (:interpolatingp
   ()
   (cond ((send ri :simulation-modep)
          (not (null timer-sequence)))
         (t ;; real robot
          (not (eq ros::simple-state ros::*simple-goal-state-done*)))))
  )

(defclass robot-interface
  :super propertied-object
  :slots (robot objects robot-state joint-action-enable warningp
                controller-type controller-actions controller-timeout
                namespace controller-table ;; hashtable :type -> (action)
                visualization-topic simulation-default-look-all-p
                joint-states-topic pub-joint-states-topic
                viewer groupname)
  :documentation "robot-interface is object for interacting real robot through JointTrajectoryAction servers and JointState topics, this function uses old nervous-style interface for historical reason. If JointTrajectoryAcion server is not found, this instance runs as simulation mode, see \"Kinematics Simulator\" view as simulated robot,

	(setq *ri* (instance robot-interface :init))
	(send *ri* :angle-vector (send *robot* :joint-angle) 2000)
	(send *ri* :wait-interpolation)
	(send *ri* :state :potentio-vector)
")

(defmethod robot-interface
  (:init
   (&rest args &key ((:robot r)) ((:objects objs)) (type :default-controller)
          (use-tf2) ((:groupname nh) "robot_multi_queue") ((:namespace ns))
          ((:joint-states-topic jst) "joint_states")
          ((:joint-states-queue-size jsq) 1)
          ((:publish-joint-states-topic pjst) nil)
          ((:controller-timeout ct) 3)
          ((:visualization-marker-topic vmt) "robot_interface_marker_array")
          ((:simulation-look-all sim-look-all) t)
          &allow-other-keys)
   "Create robot interface
- robot : class name of robot
- objects : objects to be displayed in simulation (only for simulation)
- type : method name for defining controller type
- use-tf2 : use tf2
- groupname : ros nodehandle group name
- namespace : ros nodehandle name space
- joint-states-topic (jst) : name for subscribing joint state topic
- joint-states-queue-size : queue size of joint state topic. if you have two different node publishing joint_state, better to use 2. default is 1
- publish-joint-state-topic :  name for publishing joint state topic (only for simulation)
- controller-timeout : timeout seconds for controller look-up
- visualization-marker-topic : topic name for visualize
"
   (setq joint-states-topic jst)
   (setq pub-joint-states-topic pjst)
   (setq joint-action-enable t)
   (setq controller-timeout ct)
   (setq namespace ns)
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
                     (t r)))
   (setq groupname nh)
   (unless (ros::ok)
     (ros::roseus "default_robot_interface"))
   (ros::create-nodehandle groupname)
   ;;
   (setq visualization-topic vmt)
   (ros::advertise visualization-topic visualization_msgs::MarkerArray 100)
   ;;
   (let ((start-time (now))
         (ros-now (ros::time-now))
         (wait-seconds 180)
         diff-time)
     (when (and (ros::get-param "use_sim_time" nil)
                (= 0 (send ros-now :sec))
                (= 0 (send ros-now :nsec)))
       (ros::ros-debug "[~A] /use_sim_time is TRUE, check if /clock is published" (ros::get-name))
       (while (and (= 0 (send ros-now :sec)) (= 0 (send ros-now :nsec)))
         (setq diff-time (send (now) :subtract start-time))
         (unix:usleep (* 500 1000))
         (when (> (send diff-time :second) wait-seconds)
           (ros::ros-fatal "[~A] /use_sim_time is TRUE but /clock is NOT PUBLISHED" (ros::get-name))
           (ros::ros-fatal "[~A] ~d seconds elapsed. aborting..." (ros::get-name) wait-seconds)
           (exit 1))
         (ros::ros-warn "[~A] waiting /clock... ~d seconds elapsed." (ros::get-name)
                        (+ (send diff-time :second) (* 1e-6 (send diff-time :micro))))
         (setq ros-now (ros::time-now)))
       (ros::ros-info "[~A] /clock is now published." (ros::get-name))))

   (cond
    (use-tf2
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::buffer-client :init))))
    (t
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::transform-listener :init)))))

   (ros::subscribe (if namespace (format nil "~A/~A" namespace joint-states-topic)
                     joint-states-topic) sensor_msgs::JointState
                   #'send self :ros-state-callback jsq :groupname groupname)
   ;;
   (setq controller-table (make-hash-table :size 14 :test #'eq :rehash-size 1.2))
   (setq controller-type type)
   (setq controller-actions
         (send self :add-controller controller-type :joint-enable-check t :create-actions t))
   ;;
   (when (send self :simulation-modep)
     (let ((old-viewer user::*viewer*))
       (when (and x::*display* (> x::*display* 0))
         (setq viewer (get (geo::find-viewer (send robot :name)) :pickviewer))
         (unless viewer
           (setq viewer (instance x::irtviewer :create :title (format nil "~A Kinematics Simulator" (send robot :name)) :view-name (send robot :name) :draw-floor t)))
         (send viewer :objects (list robot))
         (send self :draw-objects)
         (if old-viewer (setq user::*viewer* old-viewer)))
       (send self :objects objs)
       (ros::advertise (cond
                        (pub-joint-states-topic pub-joint-states-topic)
                        (namespace
                         (format nil "~A/~A" namespace joint-states-topic))
                        (t joint-states-topic))
                       sensor_msgs::JointState)
       (setq *top-selector-interval* 0.01)
       (setq simulation-default-look-all-p sim-look-all)
       ;; remove old timer job
       (setq *timer-job*
             (remove-if #'(lambda (x)
                      (if (listp x)
                          (let ((f (caddr x)))
                            (when (and (>= (length f) 3)
                                       (eq (class (elt f 1)) (class self))
                                       (eq (elt f 2) :robot-interface-simulation-callback))
                              (warning-message 3 "remove ~A from *timer-job*~%" f)
                              t))))
                  *timer-job*))
       (pushnew `(lambda () (send ,self :robot-interface-simulation-callback)) *timer-job*)
       (warning-message 3 "current *timer-job* is ~A~%" *timer-job*)
       ))
   self)
  ;;
  (:add-controller (ctype &key (joint-enable-check) (create-actions))
   (let (tmp-actions)
     (cond
      (create-actions
       (mapcar
        #'(lambda (param)
            (let* ((controller-action (cdr (assoc :controller-action param)))
                   (action-type (cdr (assoc :action-type param)))
                   (action (instance controller-action-client :init self
                                     (if namespace (format nil "~A/~A" namespace controller-action)
                                       controller-action) action-type
                                       :groupname groupname)))
              (push action tmp-actions)))
        (send self ctype))
       (setq tmp-actions (nreverse tmp-actions))
       ;;
       (dolist (action tmp-actions)
         (unless controller-timeout
           (ros::ros-warn "Waiting for actionlib interface forever because controller-timeout is nil"))
         (unless (and joint-action-enable (send action :wait-for-server controller-timeout))
           (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name))
           (ros::ros-warn "~C[3~CmStarting 'Kinematics Simulator'~C[0m" #x1b 49 #x1b)
           (ros::ros-warn "~C[3~Cm (If you do not intend to start Kinematics Simulator, make sure that you can run 'rostopic info /~A/goal' and 'rostopic info /~A/cancel' and check whether Subscribers exists. If there is no Subscribers, please check joint_trajectory_action node.)~C[0m" #x1b 52 (send action :name) (send action :name) #x1b)
           (ros::ros-warn "~C[3~Cm (Please also check 'rostopic info /~A/feedback' and 'rostopic info /~A/result' and check whether Publishers exists. If there is no Publishers, please check joint_trajectory_action node.)~C[0m" #x1b 52 (send action :name) (send action :name) #x1b)
           (ros::ros-warn "~C[3~Cm (If joint_trajectory_action node already exists, you might have a network problem. Please make sure that you can run 'rosnode ping JOINT_TRAJECTORY_ACTION_SERVER_NODE_NAME' and 'rosnode ping ~A')~C[0m" #x1b 52 (ros::get-name) #x1b)
           (when joint-enable-check
             (setq joint-action-enable nil)
             (return))))
       ;;
       (dolist (param (send self ctype))
         (let* ((controller-state (cdr (assoc :controller-state param)))
                (key (intern (string-upcase controller-state) *keyword-package*)))
           (ros::subscribe (if namespace (format nil "~A/~A" namespace controller-state)
                             controller-state)
                           control_msgs::JointTrajectoryControllerState
                           #'send self :set-robot-state1 key :groupname groupname)))
       )
      (t ;; not creating actions, just search
       (mapcar
        #'(lambda (param)
            (let* ((controller-action (cdr (assoc :controller-action param)))
                   (action-type (cdr (assoc :action-type param)))
                   (name (if namespace (format nil "~A/~A" namespace controller-action)
                           controller-action))
                   action)
              (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                                    controller-actions))
              (if action (push action tmp-actions))
              ))
        (send self ctype))
       (setq tmp-actions (nreverse tmp-actions))
       ))
     ;;
     (setf (gethash ctype controller-table) tmp-actions)
     tmp-actions
     ))
  ;;
  (:robot-interface-simulation-callback ()
   (let* ((joint-list (send robot :joint-list))
          (all-joint-names (send-all joint-list :name)))
     (dolist (param (send self controller-type)) ;; assuming all actions are stored in controller-type (:default-controller)
       ;(print (list 'p param))
       (let* ((joint-names (cdr (assoc :joint-names param)))
              (action-name (cdr (assoc :controller-action param)))
              (action-client (find action-name controller-actions :key #'(lambda (x) (send x :name)) :test #'string=))
              (av (send action-client :pop-angle-vector-simulation))
              )
         (when av
           (dolist (j joint-names)
             (if (find j all-joint-names :test #'string=)
                 (let ((i (position j all-joint-names :test #'string=)))
                   (send (elt joint-list i) :joint-angle (elt av i))))))
         ))
     (send self :publish-joint-state)
     ;;
     (if viewer (send self :draw-objects))
     ))
  (:publish-joint-state ;; for simulation mode (joint-action-enable is nil)
   (&optional (joint-list (send robot :joint-list)))
   (let (msg names positions velocities efforts)
     (setq msg (joint-list->joint_state joint-list))
     (send msg :header :stamp (ros::time-now))
     (when (send self :simulation-modep)
       (ros::publish (cond
                        (pub-joint-states-topic pub-joint-states-topic)
                        (namespace
                         (format nil "~A/~A" namespace joint-states-topic))
                        (t joint-states-topic))
                     msg))
     msg))

  (:angle-vector-safe
   (av tm &key (simulation) (yes) (wait t) (norm-threshold 120) (angle-threshold 80) (velocity-threshold 240))
   "send joint angle to robot with user-level confirmation. This method requires key input"
   (let* ((q (send self :state :potentio-vector))
          (jlst (send robot :joint-list))
          (diff-q (v- av q))
          (qv (scale (/ 1000.0 tm) diff-q))
          (jqlst (mapcar #'(lambda (ang j) (cons ang (send j :name)))
                         (coerce diff-q cons) jlst))
          (jvlst (mapcar #'(lambda (ang j) (cons ang (send j :name)))
                         (coerce qv cons) jlst))
          warning)
     (warn "angle-vector: ~A~%" av)
     (when (> (norm diff-q) norm-threshold)
       (setq warning t)
       (warning-message 1 "WARNING: WARNING: diff angle-vector norm is ~d [deg]~%" (norm diff-q)))
     (setq jqlst (sort jqlst #'(lambda (x y) (>= (abs (car x)) (abs (car y))))))
     (setq jqlst (remove-if-not #'(lambda (x) (> (abs (car x)) angle-threshold)) jqlst))
     (when jqlst
       (setq warning t)
       (warning-message 1 "WARNING: WARNING: typical diff angles (diff [deg] . Joint) are ~a~%" jqlst))
     (setq jvlst (sort jvlst #'(lambda (x y) (>= (abs (car x)) (abs (car y))))))
     (setq jvlst (remove-if-not #'(lambda (x) (> (abs (car x)) velocity-threshold)) jvlst))
     (when jvlst
       (setq warning t)
       (warning-message 1 "WARNING: WARNING: typical velocities (velocity [deg/sec] . Joint) are ~a~%" jvlst))
     (when (and (not simulation) (or yes (y-or-n-p "Do you send this angles to the robot? ")))
       (warn ";; send angle-vector~%")
       (send self :angle-vector av tm)
       (when wait
         (warn ";; wait interpolation~%")
         (send self :wait-interpolation))
       (warn ";; done~%")
       )
     warning))

  (:angle-vector-duration
   (start end &optional (scale 1) (min-time 1.0) (ctype controller-type))
   (let* ((unordered-joint-names 
           (flatten (mapcar #'(lambda (joint-param)
                                (cdr (assoc :joint-names joint-param)))
                            (send self ctype))))
          (joint-list (send robot :joint-list)))
     (let ((diff (coerce (v- end start) cons)))
       (let ((time-list (mapcar #'(lambda (d j)
                                    ;; check if j is included in unordered-joint-names
                                    (if (find (send j :name) unordered-joint-names
                                              :test #'string=)
                                        ;; calculate time to reach the goal for each joint
                                        ;; using (scale * (j[m/rad] / max-joint-velocity))
                                        (* scale (/ (if (derivedp j linear-joint) (* 0.001 (abs d)) (deg2rad (abs d)))
                                                    (send j :max-joint-velocity)))
                                      0))
                                diff joint-list)))
         ;; find maximum time to reach goal for all joint
         (let ((max-time (apply #'max time-list)))
           (max max-time min-time))))))
  (:angle-vector-simulation
   (av tm ctype)
   (let* ((prev-av (send robot :angle-vector)))
     (send-all (gethash ctype controller-table) :push-angle-vector-simulation av tm prev-av)))
  (:angle-vector
   (av &optional (tm nil) (ctype controller-type) (start-time 0) &key (scale 1) (min-time 1.0) (end-coords-interpolation nil) (end-coords-interpolation-steps 10))
   "Send joint angle to robot, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- av : joint angle vector [deg]
- tm : (time to goal in [msec])
   if designated tm is faster than fastest speed, use fastest speed
   if not specified, it will use 1/scale of the fastest speed .
   if :fastest is specified use fastest speed calculated from max speed
- ctype : controller method name
- start-time : time to start moving
- scale : if tm is not specified, it will use 1/scale of the fastest speed
- min-time : minimum time for time to goal
- end-coords-interpolation : set t if you want to move robot in cartesian space interpolation
- end-coords-interpolation-steps : number of divisions when interpolating end-coords
"
   (if end-coords-interpolation
     (return-from :angle-vector (send self :angle-vector-sequence (list av) (list tm) ctype start-time :scale scale :min-time min-time :end-coords-interpolation t :end-coords-interpolation-steps end-coords-interpolation-steps)))
   (setq ctype (or ctype controller-type))  ;; use default controller-type if ctype is nil
   (unless (gethash ctype controller-table)
     (warn ";; controller-type: ~A not found" ctype)
     (return-from :angle-vector))
   ;;Check and decide tm
   (let ((fastest-tm (* 1000 (send self :angle-vector-duration 
                                   (send self :state :potentio-vector) av scale min-time ctype))))
     (cond
      ;;Fastest time Mode
      ((equal tm :fast)
       (setq tm fastest-tm))
      ;;Normal Number designated Mode
      ((numberp tm)
       (if (< tm fastest-tm)
           (setq tm fastest-tm)))
      ;;Safe Mode (Speed will be 5 * fastest-time)
      ((null tm)
       (setq tm (* 5 fastest-tm)))
      ;;Error Not Good Input
      (t
       (ros::ros-error ":angle-vector tm is invalid.  args: ~A" tm)
       (error ":angle-vector tm is invalid. args: ~A" tm)))
     )
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av (send self :angle-vector-simulation av tm ctype)))
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table)))
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                start-time  ;; start time
                (list
                 (list av                                     ;; positions
                       (instantiate float-vector (length av)) ;; velocities
                       (/ tm 1000.0)))))                      ;; duration
      cacts (send self ctype)))
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)) (ctype controller-type) (start-time 0.1) &key (scale 1) (min-time 0.0) (end-coords-interpolation nil) (end-coords-interpolation-steps 10))
   "Send sequence of joint angle to robot, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- avs: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn)
- tms: sequence of duration(float) from previous angle-vector to next goal [msec],  (list tm0 tm1 ... tmn)
   if tms is atom, then use (list (make-list (length avs) :initial-element tms))) for tms
   if designated each tmn is faster than fastest speed, use fastest speed
   if tmn is nil, then it will use 1/scale of the fastest speed .
   if :fastest is specified, use fastest speed calculated from max speed
- ctype : controller method name
- start-time : time to start moving
- scale : if tms is not specified, it will use 1/scale of the fastest speed
- min-time : minimum time for time to goal
- end-coords-interpolation : set t if you want to move robot in cartesian space interpolation
- end-coords-interpolation-steps : number of divisions when interpolating end-coords
   "
   (setq ctype (or ctype controller-type))  ;; use default controller-type if ctype is nil
   (unless (gethash ctype controller-table)
     (warn ";; controller-type: ~A not found" ctype)
     (return-from :angle-vector-sequence))
   (send self :spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
         tm tm-next fastest-tm
         (vel (instantiate float-vector (length (car avs)))))
     (if (atom tms) (setq tms (make-list (length avs) :initial-element tms)))
     (cond
      ((< (length tms) (length avs))
         (nconc tms (make-list (- (length avs) (length tms)) :initial-element (car (last tms)))))
      ((> (length tms) (length avs))
       (ros::ros-warn "length of tms should be the same or smaller than avs")
       (setq tms (subseq tms 0 (length avs)))))
     (when end-coords-interpolation ;; set end-coords interpolation
       (setq min-time (/ (float min-time) end-coords-interpolation-steps))
       (let ((av-orig (send robot :angle-vector)) ;; initial av, restore at end
             (c-orig  (send robot :copy-worldcoords)) ;; inital coords, restore at end
             (av-prev-orig av-prev) ;; prev-av
	     (diff-prev (instantiate float-vector (length av-prev))) diff ;; for over 360 deg turns
             (limbs '(:larm :rarm :lleg :rleg)) ;; do not move :head and :torso by IK
             target-limbs
             (minjerk (instance minjerk-interpolator :init))
             end-coords-prev end-coords-current ec-prev ec-current
             interpolated-avs interpolated-tms
             tm pass-tm i p (ret t)) ;; if nil failed to interpolate
           ;; set prev-av
         (send robot :angle-vector av-prev)
         (setq end-coords-prev (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) limbs))
         ;; choose moved end-coords
         (setq i 0)
         (dolist (av avs)
           (send robot :angle-vector av)
           (setq end-coords-current (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) limbs))
	   (setq diff (v- (v- av (setq av (v+ av-prev (send self :sub-angle-vector av av-prev)))) diff-prev))
           (setq target-limbs nil)
	   (setq tm (elt tms i))
	   (setq pass-tm (/ tm (float end-coords-interpolation-steps)))
           (dotimes (l (length limbs))
             (setq ec-prev (elt end-coords-prev l) ec-current (elt end-coords-current l))
             (when (and ec-prev ec-current
                        (or (> (norm (send ec-prev :difference-position ec-current)) 1)
                            (> (norm (send ec-prev :difference-rotation ec-current)) (deg2rad 1))))
               (push (elt limbs l) target-limbs)))
           (send minjerk :reset :position-list (list #f(0) #f(1)) :time-list (list tm))
           (send robot :angle-vector av-prev)
           (if target-limbs
               (progn
                 (send minjerk :start-interpolation)
                 (send minjerk :pass-time pass-tm)
                 (while (progn (send minjerk :pass-time pass-tm) (send minjerk :interpolatingp))
                   (setq p (elt (send minjerk :position) 0))
                   ;; set midpoint of av as initial pose of IK
                   (send robot :angle-vector (midpoint p av-prev av))
                   (dolist (limb target-limbs)
                     (setq ec-prev (elt end-coords-prev (position limb limbs))
                           ec-current (elt end-coords-current (position limb limbs)))
                     (setq ret (and ret
                                    (send robot limb :inverse-kinematics (midcoords p ec-prev ec-current)))))
                   (push (v++ diff-prev (scale p diff) (send robot :angle-vector)) interpolated-avs)
                   (push pass-tm interpolated-tms)
                   )
               (push (v++ diff-prev diff av) interpolated-avs)
               (push pass-tm interpolated-tms)
	       )
             (progn
               (push av interpolated-avs)
               (push tm interpolated-tms)))
           (setq end-coords-prev end-coords-current)
           (setq av-prev av)
	   (setq diff-prev diff)
           (incf i)) ;; dolist (av avs)
         ;; restore states
         (setq avs (nreverse interpolated-avs) tms (nreverse interpolated-tms))
         (send robot :move-to c-orig :world)
         (send robot :angle-vector av-orig)
         (setq av-prev av-prev-orig)
         (unless ret
           (warning-message 1 ":angle-vector-sequence failed to generate end-coords-interpolation motion~%")
           (return-from :angle-vector-sequence nil))
         ))
     (prog1 ;; angle-vector-sequence returns avs
         avs
       (while avs
         (setq av (pop avs))
         (setq fastest-tm (* 1000 (send self :angle-vector-duration av-prev av scale min-time ctype)))
         (setq tm (pop tms))
         (cond
          ((equal tm :fast)
           (setq tm fastest-tm))
          ((null tm)
           (setq tm (* 5 fastest-tm)))
          ((numberp tm)
           (if (< tm fastest-tm)
               (setq tm fastest-tm)))
          (t (ros::ros-error ":angle-vector-sequence tm is invalid.  args: ~A" tm)
             (error ":angle-vector-sequence tm is invalid. args: ~A" tm)))
         (if (car tms)
             (progn
               (setq tm-next ( car tms))
               (setq fastest-tm-next (* 1000 (send self :angle-vector-duration av (car avs) scale min-time ctype)))
               (cond
                ((equal tm-next :fast)
                 (setq tm-next fastest-tm-next))
                ((null tm)
                 (setq tm (* 5 fastest-tm)))
                ((numberp tm-next)
                 (if (< tm-next fastest-tm-next)
                     (setq tm-next fastest-tm-next)))
                (t (ros::ros-error ":angle-vector-sequence tm is invalid.  args: ~A" tm)
                   (error ":angle-vector-sequence tm is invalid. args: ~A" tm)))
               )
           (setq tm-next 1)
           )
         (if (and (setq av-next (car avs)) (> tm 0) (> tm-next 0))
             (let ((v0 (send self :sub-angle-vector av av-prev))
                   (v1 (send self :sub-angle-vector av-next av)))
               (dotimes (i (length vel))
                 (setf (elt vel i)
                       (if (>= (* (elt v0 i) (elt v1 i)) 0)
                           (* 0.5 (+ (* (/ 1000.0 tm) (elt v0 i))
                                     (* (/ 1000.0 tm-next) (elt v1 i))))
                         0.0)))
               )
           (fill vel 0))
         ;; for simulation mode
         (if (send self :simulation-modep)
           (send self :angle-vector-simulation av tm ctype))
         ;;
         ;; update only joints with in current controller instead of (send robot :angle-vector av)
         (unless (send self :simulation-modep)
           (let* ((joint-names (flatten (mapcar #'(lambda (c) (cdr (assoc :joint-names c))) (send self ctype))))
                (joint-ids (mapcar #'(lambda (joint-name) (position joint-name (send robot :joint-list) :test #'(lambda (n j) (equal n (send j :name))))) joint-names)))
           (mapcar #'(lambda (name id)
                       (if (and (send robot :joint name) id (> (length av) id))
                           (send (send robot :joint name) :joint-angle (elt av id))
                         (warning-message 3 "[robot-interface.l] (angle-vector-sequence) could not find joint-name '~A' (~A) or joint-id ~A (av length ~A)~%" name (send robot :joint name) id (length av))))
                   joint-names joint-ids)))
;	 (when (send self :simulation-modep)
;	   (send self :publish-joint-state)
;	   (if viewer (send self :draw-objects)))
         (push (list av
                     (copy-seq vel)  ;; velocities
                     (/ (+ st tm) 1000.0)) ;; tm + duration
               traj-points)
         (setq av-prev av)
         (incf st tm))
       ;;
       (let ((cacts (gethash ctype controller-table)))
         (unless cacts
           (warn ";; controller-type: ~A not found" ctype)
           (return-from :angle-vector-sequence))
         (mapcar
          #'(lambda (action param)
              (send self :send-ros-controller
                    action (cdr (assoc :joint-names param)) ;; action server and joint-names
                    start-time  ;; start time
                    traj-points))
          cacts (send self ctype)))
       )))
  (:wait-interpolation (&optional (ctype) (timeout 0)) ;; controller-type
   "Wait until last sent motion is finished
- ctype : controller to be wait
- timeout : max time of for waiting
- return values is a list of interpolatingp for all controllers, so (null (some #'identity (send *ri* :wait-interpolation))) -> t if all interpolation has stopped"
   (if (send self :simulation-modep)
       (while (send self :interpolatingp)
         (send self :robot-interface-simulation-callback))
       (cond ;; real robot
         (ctype
          (let ((cacts (gethash ctype controller-table)))
            (send-all cacts :wait-for-result :timeout timeout)))
         (t (send-all controller-actions :wait-for-result :timeout timeout))))
   (send-all controller-actions :interpolatingp))
  (:interpolatingp (&optional (ctype)) ;; controller-type ;; check someone is moving
"Check if the last sent motion is executing
return t if interpolating"
    (let (cacts)
      (cond (ctype
             (setq cacts (gethash ctype controller-table)))
            (t
             (setq cacts controller-actions)))
      (some #'(lambda (a) (send a :interpolatingp)) cacts)))
  (:wait-interpolation-smooth (time-to-finish &optional (ctype))
"Return time-to-finish [msec] before the sent command is finished. Example code are:
	(dolist (av (list av1 av2 av3 av4))
	    (send *ri* :angle-vector av)
	    (send *ri* :wait-interpolation-smooth 300))
Return value is a list of interpolatingp for all controllers, so (null (some #'identity (send *ri* :wait-interpolation))) -> t if all interpolation has stopped"
    (when (not (send self :simulation-modep))
      (let ((tm (ros::time-now))
            (cacts (cond
                    (ctype (gethash ctype controller-table))
                    (t     controller-actions))))
        (while (some #'(lambda (x)
                         (and
                          (<= (send (ros::time- (send x :last-feedback-msg-stamp) tm) :to-sec) 0)
                          (let ((goal (send (x . ros::comm-state) :action-goal)))
                            (>= (send (ros::time- (ros::time+ (send goal :header :stamp)
							      (cond ((derivedp goal control_msgs::singlejointpositionactiongoal)
								     (send goal :goal :min_duration))
								    (t
								     (send (car (last (send goal :goal :trajectory :points))) :time_from_start))))
                                                  (ros::time-now)) :to-sec) 0))))
                     cacts)
          (send self :spin-once) ;; need to wait for feedback
          (send self :ros-wait 0.005))))
    (while (null (send self :interpolating-smoothp time-to-finish ctype))
      (send self :ros-wait 0.005)))
  (:interpolating-smoothp (time-to-finish &optional (ctype)) ;; controller-type
"Check if the last sent motion will stop for next time-to-finish [msec]"
   (when (send self :simulation-modep)
     (return-from :interpolating-smoothp t))
   (send self :spin-once)
   (every #'(lambda (x) (< x (/ time-to-finish 1000.0)))
          (cond
           (ctype
            (let ((cacts (gethash ctype controller-table)))
              (send-all cacts :time-to-finish)))
           (t
            (send-all controller-actions :time-to-finish)))))
  (:stop-motion (&key (stop-time 0))
   "Stop current executing motion"
   (let ((av (send self :state :potentio-vector)))
     (send self :angle-vector av stop-time)
     (send self :wait-interpolation)))
  (:cancel-angle-vector
   (&key ((:controller-actions ca))
         ((:controller-type ct))
         (wait))
   "Cancel current joint trajectory action"
   (when (null (or ca ct)) (setq ca controller-actions))
   (when ca (send-all ca :cancel-all-goals))
   (when ct (send-all (gethash ct controller-table) :cancel-all-goals))
   (when wait
     (send self :wait-interpolation)
     (send self :spin-once))
   t)
  ;;
  (:worldcoords () "Returns world coords of robot. This method uses caced data" (send robot :copy-worldcoords))
  (:torque-vector
   ()
   "Return torque vector of robot. This method uses caced data"
   (coerce (send-all (send robot :joint-list) :joint-torque) float-vector))
  (:potentio-vector () "Returns current robot angle vector. This method uses caced data" (send robot :angle-vector))
  (:reference-vector () "Returns reference joint angle vector. This method reads current state." (send self :state-vector :desired))
  (:actual-vector () "Returns current robot angle vector. This method reads current state." (send self :state-vector :actual))
  (:error-vector () "Returns error vector of the robot. This method reads current state." (send self :state-vector :error))
  (:state-vector
   (type &key ((:controller-actions ca) controller-actions)
         ((:controller-type ct) controller-type))
   (let* ((joint-list (send robot :joint-list))
          (av (instantiate float-vector (length joint-list))))
     (dolist (param (send self controller-type))
       (let* ((ctrl (cdr (assoc :controller-state param)))
              (key (intern (string-upcase ctrl) *keyword-package*))
              (msg (send self :state key))
              (idx 0))
         ;;(print (list ctrl key msg))
         (unless msg (return-from :state-vector nil))
         (dolist (jname (send msg :joint_names))
           (let ((jangle (elt (send msg type :positions) idx))
                 (j (find-if #'(lambda (jn) (string= (send jn :name) jname))
                             joint-list)))
             (incf idx)
             (when j
               (setf (elt av (position j joint-list))
                     (cond ((derivedp j linear-joint)     (* jangle 1000))
                           ((derivedp j rotational-joint) (rad2deg jangle))))))
           )))
     av))
  (:stamp () "Returns the stamp which was read at latest callback" (send self :get-robot-state :stamp))
  ;;
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (when (send self :simulation-modep)
     (return-from :send-ros-controller nil))
   (if (and warningp
	    (yes-or-no-p (format nil "~C[3~CmAre you sure to move the real robot? (~A) ~C[0m" #x1b 49 (send action :name) #x1b)))
       (return-from :send-ros-controller nil))
   (let* ((goal (send action :make-goal-instance))
	  (goal-points nil)
	  (st (if (numberp starttime)
                  (ros::time+ (ros::time-now) (ros::time starttime))
                starttime))
	  (joints (mapcar #'(lambda (x)
			      (send robot (intern (string-upcase x) *keyword-package*)))
			  joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)

     (cond
      ((equal (class goal) control_msgs::SingleJointPositionActionGoal)
       (let* ((joint (car joints))
	      (id (position joint (send robot :joint-list)))
	      (pos (deg2rad (elt (elt (car trajpoints) 0) id)))
	      )
	 (send goal :goal :position pos)
	 (send goal :goal :max_velocity 5)
	 (send self :spin-once)
	 )
       )
      (t
       (send goal :goal :trajectory :joint_names joint-names)
       (send goal :goal :trajectory :header :stamp st)
       (dolist (trajpt trajpoints)
	 (let* ((all-positions (elt trajpt 0))
		(all-velocities (elt trajpt 1))
		(duration (elt trajpt 2))
		(positions (instantiate float-vector (length joint-names)))
		(velocities (instantiate float-vector (length joint-names))))
	   (dotimes (i (length joints))
	     (let* ((joint (elt joints i))
		    (id (position joint (send robot :joint-list)))
		    p v)
	       (setq p (elt all-positions id)
		     v (elt all-velocities id))
	       (cond
		((derivedp joint rotational-joint)
		 (setq p (deg2rad p))
		 (setq v (deg2rad v)))
		(t
		 (setq p (* 0.001 p))
		 (setq v (* 0.001 v))))
	       (setf (elt positions i) p)
	       (setf (elt velocities i) v)))
	   (push (instance trajectory_msgs::JointTrajectoryPoint
			   :init
			   :positions positions
			   :velocities velocities
			   :time_from_start (ros::time duration))
		 goal-points)
	   ))
       (send self :spin-once)
       (send goal :goal :trajectory :points goal-points)
       ))
     (send action :send-goal goal)
     ))
  ;;
  (:set-robot-state1
   (key msg)
     (if (assoc key robot-state)
	 (setf (cdr (assoc key robot-state)) msg)
       (push (cons key msg) robot-state)))
  (:get-robot-state
   (key)
   (cdr (assoc key robot-state)))
  (:ros-state-callback
   (msg)
   (let ((robot-state-names (cdr (assoc :name robot-state))))
     ;; set joint name
     (cond
      (robot-state-names
       ;; merge robot-state name list
       (let ((diff (set-difference (send msg :name) robot-state-names :test #'string=)))
         (when diff
           (setq robot-state-names (append robot-state-names diff))
           ;; resize joint-state data list
           (dolist (key '(:position :velocity :effort))
             (setf (cdr (assoc key robot-state))
                   (concatenate float-vector
                                (cdr (assoc key robot-state))
                                (instantiate float-vector (- (length robot-state-names)
                                                             (length (cdr (assoc key robot-state))))))))
           ;; resize stamp-list
           (setf (cdr (assoc :stamp-list robot-state))
                 (concatenate cons
                              (cdr (assoc :stamp-list robot-state))
                              (make-list (- (length robot-state-names) (length (cdr (assoc :stamp-list robot-state))))))))
         ))
      (t
       (push (cons :name (send msg :name)) robot-state)
       (setq robot-state-names (send msg :name))
       (dolist (key '(:position :velocity :effort))
         (push (cons key (instantiate float-vector (length robot-state-names))) robot-state))
       (push (cons :stamp-list (make-list (length robot-state-names))) robot-state)))
     ;; set joint data
     (let ((joint-names (send msg :name))
           (stamp-list (cdr (assoc :stamp-list robot-state)))
           (idx 0) joint-idx joint-data data-vec)
       (dolist (key '(:position :velocity :effort))
         (setq joint-data (send msg key) idx 0)
         (when (= (length joint-names) (length joint-data))
           (setq data-vec (cdr (assoc key robot-state)))
           (dolist (jn joint-names)
             (setq joint-idx (position jn robot-state-names :test #'string=))
             (setf (elt data-vec joint-idx) (elt joint-data idx))
             (incf idx)
             ;; update stamp
             (when (eq key :position)
               (setf (elt stamp-list joint-idx) (send msg :header :stamp)))
             ))))
     (setf (cdr (assoc :name robot-state)) robot-state-names)
     ;; (dolist (key '(:name :position :velocity :effort))
     ;;   (send self :set-robot-state1 key (send msg key)))
     (send self :set-robot-state1 :stamp (send msg :header :stamp))))
  (:wait-until-update-all-joints
   (tgt-tm)
   (let ((initial-time))
     (if (equal (class tgt-tm) (class (ros::time 0)))
	 (setq initial-time (send tgt-tm :to-nsec))
       (setq initial-time (send (send (ros::time) :now) :to-nsec)))
     (while t
       (when (every #'identity (mapcar #'(lambda (ts) (> (send ts :to-nsec) initial-time)) (cdr (assoc :stamp-list robot-state))))
         (return-from nil nil))
       (if (send self :simulation-modep) (send self :robot-interface-simulation-callback)) ;; to update robot-state
       (send self :spin-once)
       )))
  (:update-robot-state
   (&key (wait-until-update nil))
   (let (joint-names positions velocities efforts)
     (send self :spin-once)
     (when wait-until-update
       (send self :wait-until-update-all-joints wait-until-update))
     ;; (unless joint-action-enable
     ;;   (return-from :update-robot-state (send robot :angle-vector)))
     (unless robot-state (return-from :update-robot-state))
     (setq joint-names (cdr (assoc :name robot-state))
	   positions (cdr (assoc :position robot-state))
	   velocities (cdr (assoc :velocity robot-state))
	   efforts (cdr (assoc :effort robot-state)))
     (let ((joint-num (length joint-names)))
       (when (not (eq joint-num (length velocities)))
	 (setq velocities (instantiate float-vector joint-num)))
       (when (not (eq joint-num (length efforts)))
	 (setq efforts (instantiate float-vector joint-num))))
     (mapcar #'(lambda (n p v e)
		 (let (j (kn (intern (string-upcase n) *keyword-package*)))
		   (when (and (find-method robot kn) (setq j (send robot kn)))
                     (send j :ros-joint-angle p)
                     ;; velocity
		     (send j :joint-velocity v)
		     ;; effort
		     (send j :joint-torque e))))
	     (coerce joint-names cons)
	     (coerce positions cons)
	     (coerce velocities cons)
	     (coerce efforts cons))))
  (:state
   (&rest args)
   "Read robot state and update internal robot instance
- :wait-until-update nil wait until joint_state is updated"
   (send self :update-robot-state
         :wait-until-update (when (position :wait-until-update args) (elt args (+ (position :wait-until-update args) 1))))
   ;; remove :wait-until-update and its argument
   (when (position :wait-until-update args)
     (setq args (append (subseq args 0 (position :wait-until-update args))
                        (cddr (memq :wait-until-update args)))))
   (unless args (return-from :state))
   (case (car args)
     ((:potentio-vector :angle-vector)
      (send robot :angle-vector))
     (:torque-vector
      (send self :torque-vector))
     (:gripper
      (apply #'send self :gripper (cdr args)))
     (t
      (let ((mm (find (car args) (remove-if-not #'(lambda (x) (substringp "-VECTOR" (string x))) (send self :methods)))))
        (if mm ;; if xx-vector method exists
            (send* self mm (cdr args))
          ;; digging arguments in robot-state and return nil if not found
          ;; (send *ri* :state :odom :pose :x)
          (let ((cur robot-state))
            (dolist (key args cur)
              (setq cur (cdr (assoc key cur))))))
        ))))
  ;;
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/follow_joint_trajectory_action")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  ;;
  (:sub-angle-vector (v0 v1)
   (let ((ret (v- v0 v1))
	 (joint-list (send robot :joint-list))
	 (i 0) j)
     (while (setq j (pop joint-list))
       (when (and (= (send j :min-angle) *-inf*) (= (send j :max-angle) *inf*))
	   (setf (elt ret i) (mod (elt ret i) 360.0))
	   (cond ((> (elt ret i) 180.0)
		  (setf (elt ret i) (- (elt ret i) 360.0)))
		 ((< (elt ret i) -180.0)
		  (setf (elt ret i) (+ (elt ret i) 360.0)))))
       (incf i))
     ret))
  ;;
  (:robot  (&rest args) (forward-message-to robot args))
  (:viewer (&rest args) (forward-message-to viewer args))
  (:objects (&optional objs)
	    (when objs
	      (setq objects (mapcar #'(lambda (o) (let ((p (send o :parent))) (if p (send p :dissoc o)) (setq o (copy-object o)) (if p (send p :assoc o)) o)) objs))
	      (mapcar #'(lambda (o) (setf (get o :GL-DISPLAYLIST-ID) nil)) (x::draw-things objects))
	      (when viewer
                (send viewer :objects (append (list robot) objects))
                (send self :draw-objects)))
	    objects)
  (:set-simulation-default-look-at (look-at-p)
   (setq simulation-default-look-all-p look-at-p))
  (:draw-objects (&key look-all)
   (when viewer
     (when (or simulation-default-look-all-p look-all)
       (send viewer :look-all (send (geo::make-bounding-box (flatten (send-all (x::draw-things robot) :vertices))) :grow 0.3)))
     (send viewer :draw-objects)
     (x::window-main-one)))
  (:find-object (name-or-obj) "Returns objects with given name or object of the same name."
   (let ((name (if (derivedp name-or-obj cascaded-link) (send name-or-obj :name) name-or-obj)))
     (append
      (mapcan #'(lambda (x) (if (derivedp x scene-model) (send x :find-object name) nil)) objects)
      (mapcan #'(lambda (x) (if (string= name (send x :name)) (list x))) objects))))
  ;;
  (:joint-action-enable (&optional (e :dummy)) (if (not (eq e :dummy)) (setq joint-action-enable e)) joint-action-enable)
  (:simulation-modep () "Check if in simulation mode" (null joint-action-enable))
  (:warningp (&optional (w :dummy)) "When in warning mode, it wait for user's key input before sending angle-vector to the robot" (if (not (eq w :dummy)) (setq warningp w)) warningp)
  (:spin-once () (ros::spin-once groupname))
  (:send-trajectory (joint-trajectory-msg
                     &key ((:controller-type ct) controller-type) (starttime 1) &allow-other-keys)
   (unless (gethash ct controller-table)
     (warn ";; controller-type: ~A not found" ct)
     (return-from :send-trajectory))
   (mapcar
    #'(lambda (action param)
        (send self :send-trajectory-each
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              joint-trajectory-msg
              starttime))
    (gethash ct controller-table) (send self ct)))
  (:send-trajectory-each
   (action joint-names traj &optional (starttime 0.2))
   (let* ((jnames (send traj :joint_names))
          (ilst (mapcar #'(lambda (jn) (position jn jnames :test #'string=)) joint-names))
          points-lst)
     (when (some #'identity ilst)
       (setq ilst (mapcar #'(lambda (jn)
                              (let ((p (position jn jnames :test #'string=)))
                                (unless p
                                  (setq p (send robot (intern (string-upcase jn) *keyword-package*))))
                                p))
                          joint-names))
       (dolist (p (send traj :points))
         (let ((poss (send p :positions))
               (vels (send p :velocities))
               (effs (send p :accelerations))
               plst vlst elst)
           (dolist (i ilst)
             (cond
              ((numberp i)
               (push (elt poss i) plst)
               (if (and vels (> (length vels) i)) (push (elt vels i) vlst))
               (if (and effs (> (length effs) i)) (push (elt effs i) elst)))
              (t
               (push (send i :ros-joint-angle) plst)
               (if vels (push 0 vlst))
               (if effs (push 0 elst))
               (ros::ros-warn ";; trajectory contains lacking joint names")
               )))
           (push
            (instance trajectory_msgs::JointTrajectoryPoint :init
                      :positions (coerce (nreverse plst) float-vector)
                      :velocities (if vels (coerce (nreverse vlst) float-vector))
                      :accelerations (if effs (coerce (nreverse elst) float-vector))
                      :time_from_start (send p :time_from_start)) points-lst)
           ))
       (let ((goal (send action :make-goal-instance))
             (st (ros::time+ (ros::time-now) (ros::time starttime))))
         (send goal :header :stamp (ros::time-now))
         (send goal :header :seq 1)
         (send goal :goal :trajectory :header :stamp st)
         (send goal :goal :trajectory :header :seq 1)
         (send goal :goal :trajectory :joint_names joint-names)
         (send goal :goal :trajectory :points (nreverse points-lst))
         (send self :spin-once)
         (send action :send-goal goal))
       (apply-trajectory_point joint-names (car (last points-lst)) robot)
       ) ;;; /when ilst
     ))
  (:ros-wait (tm &key (spin) (spin-self) (finish-check) &allow-other-keys) ;; just wait in ros-time
   (ros::rate 100) ;;
   (let ((st (ros::time-now)))
     (if spin (ros::spin-once))
     (if spin-self (send self :spin-once))
     (while t
       (when finish-check
         (if (funcall finish-check self)
           (return)))
       (let ((tdiff (ros::time- (ros::time-now) st)))
         (if (> (send tdiff :to-sec) tm)
             (return)))
       (if spin (ros::spin-once))
       (if spin-self (send self :spin-once))
       (ros::sleep)))
   t)
  ;;
  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   "move robot toward x, y, degree and wait to reach that goal. Return t if reached or nil if fail
    the robot moves relative to current position.
    using [m] and [degree] is historical reason from original hrpsys code"
   (error "subclass's responsibility for (send ~s :go-pos)~%" self))
  (:go-pos-no-wait
   (x y &optional (d 0)) ;; [m] [m] [degree]
   "no-wait version of :go-pos. This function is always assumed to return t"
   (error "subclass's responsibility for (send ~s :go-pos-no-wait)~%" self))
  (:go-wait
   ()
   "wait until :go-pos-no-wait reached the goal. Return t if reached or nil if fail"
   (error "subclass's responsibility for (send ~s :go-wait)~%" self))
  (:go-velocity
   (x y d ;; [m/sec] [m/sec] [rad/sec]
    &optional (msec 1000) ;; msec is total animation time [msec]
    &key (stop t) (wait))
   "move robot at given speed for given msec.
    if stop is t, robot stops after msec, use wait t for blocking call
    return nil if aborted while waiting by enabling :wait, otherwise return t"
   (error "subclass's responsibility for (send ~s :go-velocity)~%" self))
  (:go-stop
   ()
   "stop go-velocity. Return t if robot successfully stops, otherwise return nil"
   (error "subclass's responsibility for (send ~s :stop)~%" self))
  (:gripper
   (&rest args)
   "get information about gripper"
   (error "This method is responsible to sub class~%"))
  ) ;; robot-interface
;; ros visualization methods
(defmethod robot-interface
  (:joint-trajectory-to-angle-vector-list
   (move-arm joint-trajectory
             &key ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50)
             (show-trajectory t) (send-trajectory t)
             ((:speed-scale speed-scale) 1.0) &allow-other-keys)
   (let* (joint-trajectory-points
          joints avs tms  (tm (ros::time -1)) link-coords-list
          (org-av (send robot :angle-vector))
          (dae-link-list
           (send self :find-descendants-dae-links (send robot move-arm :root-link))))
     ;; JointTrajectory to angle-vector
     (if joint-trajectory
         (setq joint-trajectory-points (send joint-trajectory :points)
               joints (mapcar #'(lambda (x)
                                  (send robot (intern (string-upcase x) *keyword-package*)))
                              (send joint-trajectory :joint_names))))
     ;;
     (if (= (length joint-trajectory-points) 0)
         (return-from :joint-trajectory-to-angle-vector-list nil))
     (dolist (point joint-trajectory-points)
       (mapc '(lambda (ajoint aposition)
                (send ajoint :ros-joint-angle aposition))
             joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (when (cadr avs)
         (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
         (when (> diff-sum diff-thre)
           (setq diff-sum 0)
           (when show-trajectory
             (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
                   link-coords-list))))
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000 speed-scale) tms)
       (setq tm (send point :time_from_start))
       )
     (when show-trajectory
       (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
             link-coords-list)
       ;; send visualization_msgs to rviz
       ;; (send self :show-mesh-traj-with-color (send-all (flatten (send-all link-list :bodies)) :name) link-coords-list)
       (send self :show-mesh-traj-with-color
             (flatten (send-all dae-link-list :bodies))
             link-coords-list :lifetime (+ (/ (apply #'+ tms) 1000.0) 10)))
     (cond
      (send-trajectory
       (send robot :angle-vector (car (last avs)))
       ;; send to *ri*
       (send self :angle-vector-sequence (setq avs (reverse avs)) (setq tms (reverse tms))))
      (t
       (send robot :angle-vector org-av)
       ))
     (if joint-trajectory (list (reverse avs) (reverse tms)))
     ))
  (:show-goal-hand-coords (coords move-arm)
   (let* ((gripper-bodies
           (flatten (send-all (send robot move-arm :gripper :links) :bodies)))
          (gripper-coords
           (mapcar #'(lambda (gripper-link)
                       (send (send coords :copy-worldcoords) :transform
                             (send (send robot move-arm :end-coords) :transformation gripper-link)
                             :local))
                   gripper-bodies)))
     (send self :show-mesh-traj-with-color
           gripper-bodies (list gripper-coords)
           :lifetime 0 :color #f(1 0 1) :ns "hand_traj")
     (list gripper-bodies gripper-coords)))
  (:find-descendants-dae-links (l)
   (unless l (return-from :find-descendants-dae-links nil))
   (append (list l)
           (mapcan #'(lambda (x) (send self :find-descendants-dae-links x)) (send l :child-links))))
  (:show-mesh-traj-with-color
   (link-body-list link-coords-list &key ((:lifetime lf) 20)
                   (ns "robot_traj") ((:color col) #f(0.5 0.5 0.5)))
   (let ((msg (instance visualization_msgs::MarkerArray :init))
         (header (instance std_msgs::header :init
                           :stamp (ros::time-now)
                           :frame_id (send (car (send robot :links)) :name)))
         (l (length link-coords-list)) markers-list alpha-list
         (base-cds (send (car (send robot :links)) :copy-worldcoords)))
     (setq base-cds (send base-cds :inverse-transformation))
     (dotimes (i l)
       (push (+ (/ (/ (1+ i) 2.0) l) 0.5) alpha-list))
     (dotimes (i l)
       (let (mrk markers)
         (mapcar #'(lambda (abody acoords)
                     (setq acoords (send (send acoords :copy-worldcoords) :transform base-cds :world))
                     (cond ((send abody :name)
                            (setq mrk (mesh->marker-msg
                                       acoords
                                       (send abody :name) ;; body would have :name as filename of original mesh
                                       header
                                       :mesh_use_embedded_materials nil :color col :alpha (elt alpha-list i))))
                           (t
                            (setq mrk (object->marker-msg
                                       abody
                                       header
                                       :coords acoords
                                       :color col
                                       :alpha (elt alpha-list i)
                                       ))))
                     (send mrk :lifetime (ros::time lf))
                     (send mrk :ns ns)
                     (push mrk markers))
                 link-body-list
                 (elt link-coords-list i))
         (push markers markers-list)))
     (setq markers-list (flatten markers-list))
     (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
     (send msg :markers markers-list)
     (ros::publish visualization-topic msg)
     ))
  (:nod
   (&key (angle 40) (time 3000))
   "Nod robot head"
   (let (prev-neck-p)
     (send self :update-robot-state)
     (send self :robot :angle-vector (send self :state :reference-vector))
     (setq prev-neck-p (send self :robot :head :neck-p :joint-angle))
     (send self :robot :head :neck-p :joint-angle angle)
     (send self :angle-vector (send self :robot :angle-vector) time)
     (send self :wait-interpolation)
     (send self :robot :head :neck-p :joint-angle prev-neck-p)
     (send self :angle-vector (send self :robot :angle-vector) time)
     (send self :wait-interpolation)
     )
   )
  (:eus-mannequin-mode
   (tmp-robot limb &key ((:time tm) 1000) ((:viewer vwr) *viewer*))
   "Euslisp version mannequin mode.
    In every loop, :angle-vector is continuously updated.
    If you want to stop this mode, press Enter.
- tmp-robot : argument for robot instance and angle-vector of tmp-robot is updated in this method.
- limb : mannequin mode limb such as :rarm, :larm, :rleg, and :lleg.
- tm : angle-vector update time [ms]. 1000 by default."
   (objects (list tmp-robot))
   (warn ";; Start Euslisp mannequin mode.~%")
   (warn ";; To stop this mode, press Enter.~%")
   (do-until-key
    (send self :state)
    (send tmp-robot limb :angle-vector (send robot limb :angle-vector))
    (send vwr :draw-objects)
    (send self :angle-vector (send tmp-robot :angle-vector) tm)
    (send self :wait-interpolation)
    )
   (warn ";; Stop Euslisp mannequin mode.~%")
   )
  )
;;
;; for text-to-speech services
(defmethod robot-interface
  (:play-sound (sound &key arg2 (topic-name "robotsound") wait)
   (funcall #'play-sound sound :arg2 arg2 :topic-name topic-name :wait wait))
  (:speak (text &key (lang "") (topic-name "robotsound") wait)
   (send self :play-sound text
         :topic-name topic-name
         :wait wait
         :arg2 (if (keywordp lang) (string-downcase lang) lang)))
  (:speak-en (text &key (topic-name "robotsound") wait)
   (send self :speak text :topic-name topic-name :wait wait))
  (:speak-jp (text &key (topic-name "robotsound_jp") wait)
   (send self :speak text :lang :ja :topic-name topic-name :wait wait)))
;;
(defclass robot-move-base-interface
  :super robot-interface
  :slots (move-base-action move-base-trajectory-action
          move-base-goal-msg move-base-goal-coords  move-base-goal-map-to-frame
          base-frame-id
          odom-topic move-base-trajectory-joint-names
          go-pos-unsafe-goal-msg
          current-goal-coords)) ;; for simulation-callback
(defmethod robot-move-base-interface
  (:init
   (&rest args &key
          (move-base-action-name "move_base") ((:base-frame-id base-frame-id-name) "base_footprint")
          (base-controller-action-name "/base_controller/follow_joint_trajectory")
          (base-controller-joint-names (list "base_link_x" "base_link_y" "base_link_pan"))
	  ((:odom-topic odom-topic-name) "/base_odometry/odom") &allow-other-keys)
   (prog1 (send-super* :init args)
     (setq base-frame-id base-frame-id-name)
     (setq odom-topic odom-topic-name)
     (setq move-base-action (instance ros::simple-action-client :init
                                      move-base-action-name move_base_msgs::MoveBaseAction
                                      :groupname groupname))
     (when base-controller-action-name
       (setq move-base-trajectory-action
             (instance ros::simple-action-client :init
                       base-controller-action-name
                       control_msgs::FollowJointTrajectoryAction
                       :groupname groupname))
       (unless (send move-base-trajectory-action :wait-for-server 3)
         (ros::ros-warn "move-base-trajectory-action is not found")
         (setq move-base-trajectory-action nil))
       (setq move-base-trajectory-joint-names base-controller-joint-names))
     (ros::subscribe odom-topic-name nav_msgs::Odometry
		     #'send self :odom-callback :groupname groupname)
     ))
  ;;
  (:odom-callback
   (msg)
   (let ((parsed
	  (list
	   (cons :stamp (send msg :header :stamp))
	   (cons :pose (ros::tf-pose->coords (send msg :pose :pose)))
	   (cons :velocity (float-vector
			    (* 1000 (send msg :twist :twist :linear :x))
			    (* 1000 (send msg :twist :twist :linear :y))
			    (send msg :twist :twist :angular :z))))))
     (send self :set-robot-state1 :odom parsed)))
  ;;
  (:go-stop (&optional (force-stop t))
   (when joint-action-enable
     (send move-base-action :cancel-all-goals)
     (prog1
         (send move-base-trajectory-action :cancel-all-goals)
       (if force-stop (send self :go-velocity 0 0 0)))
     ))

  (:make-plan
   (st-cds goal-cds &key (start-frame-id "world") (goal-frame-id "world"))
   (let ((req (instance nav_msgs::GetPlanRequest :init))
	 (tm (ros::time-now))
	 map-to-frame
	 map-to-base
	 res
	 plan-cds-seq)
     (setq map-to-base (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
     (setq map-to-frame (send *tfl* :lookup-transform "map" start-frame-id (ros::time 0)))
     (send req :start :header :stamp)
     (send req :start :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send req :start :header :frame_id "map")
	   (send req :start :pose (ros::coords->tf-pose (send (send st-cds :copy-worldcoords) :transform map-to-frame :world))))
       (progn
	   (send req :start :header :frame_id frame-id)
	   (send req :start :pose (ros::coords->tf-pose st-cds))))
     (setq map-to-frame (send *tfl* :lookup-transform "map" goal-frame-id (ros::time 0)))
     (send req :goal :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send req :goal :header :frame_id "map")
	   (send req :goal :pose (ros::coords->tf-pose (send (send goal-cds :copy-worldcoords) :transform map-to-frame :world))))
       (progn
	 (send req :start :header :frame_id frame-id)
	 (send req :start :pose (ros::coords->tf-pose st-cds))))

     (setq res (ros::service-call "/move_base_node/make_plan" req))
     (unless (send res :plan :poses)
       (return-from :make-plan nil))
     (setq plan-cds-seq (mapcar #'(lambda (p-stamped)
				    (let ((cds (ros::tf-pose->coords (send p-stamped :pose))))
				      (if map-to-base
					  (progn
					    (send cds :transform (send map-to-base :inverse-transformation) :world)
					;;(send cds :transform (send *pr2* :copy-worldcoords) :world))
					    )
					cds)))
				(send res :plan :poses)))
     plan-cds-seq))
  ;;
  ;;
  (:move-to
   (coords &rest args &key (no-wait nil) &allow-other-keys)
   (send* self :move-to-send coords args)
   (if (not no-wait)
       (send* self :move-to-wait args) t))
  (:move-to-send
   (coords &key (frame-id "world") (wait-for-server-timeout 5) (count 0) &allow-other-keys)
   (setq move-base-goal-msg (instance move_base_msgs::MoveBaseActionGoal :init))
   (setq move-base-goal-coords coords)
   (if (send self :simulation-modep)
       (let ()
         (cond ((equal frame-id base-frame-id)
                (setq current-goal-coords (send coords :transform robot :world))) ;; for simulation-callback
               (t
                (setq current-goal-coords (send coords :copy-worldcoords)))) ;; for simulation-callback
         (return-from :move-to-send)))
   (let (ret (count 0) (tm (ros::time-now))
	     (map-to-frame (send *tfl* :lookup-transform "map" frame-id (ros::time 0))))
     ;; store in slot variable for :move-to-wait
     (setq move-base-goal-map-to-frame map-to-frame)
     ;;
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from :move-to-send))
     ;;
     (send move-base-goal-msg :header :stamp tm)
     (send move-base-goal-msg :goal :target_pose :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send move-base-goal-msg :goal :target_pose :header :frame_id "map")
	   (send move-base-goal-msg :goal :target_pose :pose
		 (ros::coords->tf-pose (send (send coords :copy-worldcoords) :transform map-to-frame :world))))
       (progn ;; fail to find "/map" to frame_id
	   (send move-base-goal-msg :goal :target_pose :header :frame_id frame-id)
	   (send move-base-goal-msg :goal :target_pose :pose (ros::coords->tf-pose coords))
	 ))
     (send move-base-goal-msg :header :seq count)
     (send move-base-goal-msg :goal :target_pose :header :seq count)
     (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
     (send move-base-action :send-goal move-base-goal-msg)
     move-base-goal-msg))
  (:move-to-wait
   (&rest args &key (retry 10) (frame-id "world") &allow-other-keys)
   (let (ret (count 0) (tm (ros::time-now))
	     (map-to-frame move-base-goal-map-to-frame)
             (coords move-base-goal-coords))
     (when (send self :simulation-modep)
         ;; wait for-result
         (while current-goal-coords
           (send self :robot-interface-simulation-callback))
         (return-from :move-to-wait t)) ;; simulation-modep
     (if (null move-base-goal-msg) (return-from :move-to-wait nil))
     (while (and (null ret) (<= count retry))
       (when (> count 0) ;; retry
         (send self :clear-costmap)
         (send move-base-goal-msg :header :seq count)
         (send move-base-goal-msg :goal :target_pose :header :seq count)
         (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
         (send move-base-action :send-goal move-base-goal-msg))
       (send move-base-action :wait-for-result)
       (when (eq (send move-base-action :get-state)
                 actionlib_msgs::GoalStatus::*preempted*)
         (setq ret nil)
         (return))
       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
           (setq ret t))
       (incf count))
     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
     ;;
     (when ret
       (let (diff diff-len current-coords lret map-goal-coords)
         ;;
         (setq map-goal-coords
               (if (string= frame-id base-frame-id)
                   (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                 (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                       :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
         (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
         (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
         (when (null lret)
           (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
           (setq move-base-goal-msg nil)
           (return-from :move-to-no-wait nil))
         (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
         (setq diff (send current-coords :transformation map-goal-coords))
         (ros::ros-warn ":move-to current-coords  ~A" current-coords)
         (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
         (ros::ros-warn "          error-coords   ~A" diff)
         (ros::ros-warn "         target-coords   ~A" coords)
         ;;
         (dotimes (i 2)
           (if (< (setq diff-len (norm (subseq (send diff :worldpos) 0 2))) 200) ;; move_base thre = 200mm
               (let* ((msec (* diff-len 10))
                      (x (/ (elt (send diff :worldpos) 0) msec))
                      (y (/ (elt (send diff :worldpos) 1) msec))
                      (d (/ (elt (car (rpy-angle (send diff :worldrot))) 0) (/ msec 1000))))
                 (ros::ros-warn ":move-to -> :go-velocity x:~A y:~A d:~A msec:~A" x y d msec)
                 (unix:usleep (* 400 1000)) ;; 400ms ???
                 (let ((acret (send self :go-velocity x y d msec :wait t)))
                   (unless acret
                     (setq move-base-goal-msg nil)
                     (return-from :move-to-wait nil)))
                 ;;(unix::usleep (* (round msec) 1000)) ;; why time wait
                 )
             (progn
               (ros::ros-error "too far from goal position ~A mm (> 200mm)" diff-len)
	       ;; move-to succeeded but away from 200 mm
	       (ros::ros-error ":move-to try to send /move_base_simple/goal")
	       (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
	       (send move-base-goal-msg :goal :target_pose :header :seq (1+ count))
	       (ros::publish "/move_base_simple/goal" (send move-base-goal-msg :goal :target_pose))
	       (unix:sleep 3)
               (setq move-base-goal-msg nil)
               (return-from :move-to-wait nil)
               ))
           ;;
           (setq map-goal-coords
                 (if (string= frame-id base-frame-id)
                     (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                   (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                         :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
           (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
           (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
           (when (null lret)
             (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
             (setq move-base-goal-msg nil)
             (return-from :move-to-wait nil))
           (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
           (setq diff (send current-coords :transformation map-goal-coords))
           (ros::ros-warn ":move-to current-coords  ~A" current-coords)
           (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
           (ros::ros-warn "          error-coords   ~A" diff)
           (ros::ros-warn "         target-coords   ~A" coords)
           ) ;;  (do (i 2)
         ))
     (setq move-base-goal-msg nil) ;; :move-to-wait has been called
     ret))

  (:go-waitp
   ()
   (send self :spin-once) ;; update
   (when (send self :simulation-modep)
     (return-from :go-waitp (not (null current-goal-coords))))
   (cond
    ((or
      (equal (send move-base-action :get-state) ros::*simple-goal-state-active*)
      (equal (send move-base-trajectory-action :get-state) ros::*simple-goal-state-active*))
     (return-from :go-waitp t))
    (t
     (return-from :go-waitp nil))))

  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id base-frame-id)
     ))
  (:go-pos-no-wait
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id base-frame-id :no-wait t)
     ))
  (:go-wait
   ()
   (let ()
     (send self :move-to-wait :retry 1 :frame-id base-frame-id :no-wait nil)
     ))
  (:go-velocity
   (x y d ;; [m/sec] [m/sec] [rad/sec]
    &optional (msec 1000) ;; msec is total animation time [msec]
    &key (stop t) (wait))
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
	   (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x msec) (* y msec) 0)) ;; [m/sec] * [msec] = [mm]
       (send coords :rotate (deg2rad (/ (* d msec) 1000.0)) :z) ;; [rad/sec] * [msec] / 1000 = [rad]->(deg2rad)->[deg]
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	   ((> curr-tm msec))
	 (send robot :newcoords (midcoords (/ curr-tm (float msec)) orig-coords coords))
	 (if viewer (send self :draw-objects))))
     (return-from :go-velocity t))
   (unless move-base-trajectory-action
     (ros::ros-warn ":go-velocity is disabled. (move-base-trajectory-action is not found)")
     (return-from :go-velocity t))
   (setq x (* x (/ msec 1000.0))
         y (* y (/ msec 1000.0))
         d (* d (/ msec 1000.0)))
   (let ((goal (send self :move-trajectory x y d msec :stop stop)))
     (prog1
         (send move-base-trajectory-action :send-goal goal)
       (if wait (send move-base-trajectory-action :wait-for-result)))
     ))
  (:go-pos-unsafe (&rest args)
    (send* self :go-pos-unsafe-no-wait args)
    (send self :go-pos-unsafe-wait))
  (:go-pos-unsafe-no-wait
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (ros::ros-info "go-pos-unsafe (x y d) = (~A ~A ~A)" x y d)
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
           (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x 1000) (* y 1000) 0))
       (send coords :rotate (deg2rad d) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
           ((> curr-tm 1000))
         (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
         (if viewer (send self :draw-objects))))
     (return-from :go-pos-unsafe-no-wait t))
   (unless move-base-trajectory-action
     (ros::ros-warn ":go-pose-unsafe-no-wait is disabled. (move-base-trajectory-action is not found)")
     (return-from :go-pos-unsafe-no-wait t))
   (let ((maxvel 0.295) (maxrad 0.495)
         msec)
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                             (/ (abs (deg2rad d)) (* maxrad 0.8))
                             1.0)))
     (setq go-pos-unsafe-goal-msg (send self :move-trajectory
                                        x y (deg2rad d) msec
                                        :stop t))
     (send move-base-trajectory-action :send-goal go-pos-unsafe-goal-msg)
     ))
  (:go-pos-unsafe-wait ()
   (let (x y d msec step goal (maxvel 0.295) (maxrad 0.495) (counter 0))
     (if (null go-pos-unsafe-goal-msg) (return-from :go-pos-unsafe-wait nil))
     (unless move-base-trajectory-action
       (ros::ros-warn ":go-pose-unsafe-wait is disabled. (move-base-trajectory-action is not found)")
       (return-from :go-pos-unsafe-wait t))
     (while (< counter 3) ;; magic number 3 times
       (let ((acret
              (send move-base-trajectory-action :wait-for-result)))
         (unless acret
           (return-from :go-pos-unsafe-wait nil))
         (send move-base-trajectory-action :spin-once))
       (send self :spin-once)
       (send self :spin-once)
       (let ((goal-position (send (elt (send go-pos-unsafe-goal-msg :goal :trajectory :points) 1) :positions)) ;; goal
             (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
             (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
             diff-position v)
         (setq diff-position (v- goal-position (v+ odom-pos (float-vector 0 0 odom-angle))))
         (setq v (v- (rotate-vector (float-vector (elt diff-position 0) (elt diff-position 1) 0)
                                    (- odom-angle) :z)
                     (float-vector 0 0 odom-angle)))
         (setq x (elt v 0)
               y (elt v 1)
               d (rad2deg (elt diff-position 2)))
         (if (> d (* 360 0.8)) (setq d (- d 360)))
         (if (< d (* -360 0.8)) (setq d (+ d 360)))
         (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                                 (/ (abs (deg2rad d)) (* maxrad 0.8)))))
         (setq msec (max msec 1000))
         (setq step (/ 1000.0 msec))
         (ros::ros-info "                diff-pos ~A ~A, diff-angle ~A" x y d)

         (if (and (<= (sqrt (+ (* x x) (* y y)))  0.025)
                  (<= (abs d) 2.5) ;; 3 cm and 3deg
                  (/= counter 0))  ;; try at least 1 time
             (progn
               (setq go-pos-unsafe-goal-msg nil) ;; go-pos-unsafe-wait has been called
               (return-from :go-pos-unsafe-wait t))
           )
         (setq go-pos-unsafe-goal-msg (send self :move-trajectory (* x step) (* y step)
                                            (* (deg2rad d) step) msec :stop t))
         (send move-base-trajectory-action :send-goal go-pos-unsafe-goal-msg)
         ) ;; let
       (incf counter)
       ) ;; while
     (setq go-pos-unsafe-goal-msg nil) ;; go-pos-unsafe-wait has been called
     t))
  ;;
  (:move-trajectory-sequence
   (trajectory-points time-list &key (stop t) (start-time) (send-action nil))
   "Move base following the trajectory points at each time points
    trajectory-points [ list of #f(x y d) ([m] for x, y; [rad] for d) ]
    time-list [list of time span [msec] ]
    stop [ stop after msec moveing ]
    start-time [ robot will move at start-time [sec or ros::Time] ]
    send-action [ send message to action server, it means robot will move ]"
   (send self :spin-once)
   (labels ((normalize-angle-positive (d)
              (mod (+ (mod d 2pi) 2pi) 2pi))
            (normalize-angle (d)
              (let ((a (normalize-angle-positive d)))
                (if (> a pi) (- a 2pi) a)))
            (shortest-angular-distance (from to)
              (normalize-angle (- to from))))
     (let ((odom-cds (send self :state :odom :pose))
           (msg (instance trajectory_msgs::JointTrajectory :init))
           (goal (instance control_msgs::FollowJointTrajectoryActionGoal :init))
           (cur-time 0) (nxt-time 0)
           cds-lst pts-msg-lst
           cur-cds nxt-cds)
       (send msg :joint_names move-base-trajectory-joint-names)
       ;; parse start-time
       (cond
         ((numberp start-time)
          (send msg :header :stamp (ros::time+ (ros::time-now) (ros::time start-time))))
         (start-time (send msg :header :stamp start-time))
         (t (send msg :header :stamp (ros::time-now))))
       (setq cds-lst
             (mapcar #'(lambda (pt)
                         (let ((cds (make-cascoords :coords odom-cds)))
                           (send cds :translate (float-vector
                                                 (* (elt pt 0) 1000.0)
                                                 (* (elt pt 1) 1000.0) 0))
                           (send cds :rotate (elt pt 2) :z)
                           cds))
                     trajectory-points))
       (setq cur-cds odom-cds
             nxt-cds (pop cds-lst)
             nxt-time (pop time-list)
             cur-yaw (caar (send cur-cds :rpy-angle))
             nxt-yaw 0)
       (while nxt-cds
         (unless (ros::ok) (return-from :move-trajectory-sequence nil))
         (let* ((tra  (send cur-cds :transformation nxt-cds))
                (rot (send cur-cds :rotate-vector (send tra :pos)))
                (diff-yaw (shortest-angular-distance
                           (caar (send cur-cds :rpy-angle))
                           (caar (send nxt-cds :rpy-angle)))))
           (push
            (instance trajectory_msgs::JointTrajectoryPoint :init
                      :positions  (float-vector
                                   (/ (elt (send cur-cds :pos) 0) 1000)
                                   (/ (elt (send cur-cds :pos) 1) 1000)
                                   cur-yaw)
                      :velocities (float-vector
                                   (/ (elt rot 0) nxt-time)
                                   (/ (elt rot 1) nxt-time)
                                   (/ (caar (send tra :rpy-angle)) nxt-time 0.001))
                      :time_from_start (ros::time cur-time))
            pts-msg-lst)

           (incf cur-time (/ nxt-time 1000.0))
           (setq cur-cds nxt-cds
                 nxt-cds (pop cds-lst)
                 nxt-time (pop time-list)
                 cur-yaw (if (> (caar (send tra :rpy-angle)) 0)
                             (+ cur-yaw (abs diff-yaw))
                             (- cur-yaw (abs diff-yaw)))))) ;; while
       (push ;; the last point
        (instance trajectory_msgs::JointTrajectoryPoint :init
                  :positions  (float-vector
                               (/ (elt (send cur-cds :pos) 0) 1000)
                               (/ (elt (send cur-cds :pos) 1) 1000)
                               cur-yaw)
                  :velocities (if stop
                                  (float-vector 0 0 0)
                                  (send (car pts-msg-lst) :velocities))
                  :time_from_start (ros::time cur-time))
        pts-msg-lst)
       (send msg :points (reverse pts-msg-lst))
       (send goal :goal :trajectory msg)
       (when send-action
         (unless move-base-trajectory-action
           (ros::ros-error "send-action is t, but move-base-trajectory-action is not found")
           (return-from :move-trajectory-sequence nil))
         (send move-base-trajectory-action :send-goal goal)
         (if (send move-base-trajectory-action :wait-for-result)
             (return-from :move-trajectory-sequence
               (send move-base-trajectory-action :get-result))
             (return-from :move-trajectory-sequence nil)))
       goal)))
  ;;
  (:move-trajectory
   (x y d &optional (msec 1000) &key (stop t) (start-time) (send-action nil))
   "x [m] y [m] d [rad] msec [milli second (default: 1000)]
    stop [ stop after the base reached the goal if T (default: T)]
    start-time [ robot starts to move from `start-time` (default: now) ]
    send-action [ send the goal to action server if enabled, otherwise just returns goal without sending (default: nil) ]"
   (send self :move-trajectory-sequence
         (list (float-vector x y d))
         (list msec)
         :stop stop :start-time start-time :send-action send-action))
  ;;
  (:state
   (&rest args)
   (prog1
       (send-super* :state args)
     (case (car args)
       (:worldcoords
	(unless joint-action-enable
	  (return-from :state (send self :worldcoords)))
	(return-from :state (send *tfl* :lookup-transform (or (cadr args) "map") base-frame-id (ros::time)))))))
  ;;
  (:robot-interface-simulation-callback
   ()
   (when current-goal-coords
     (let* ((orig-coords (send robot :copy-worldcoords))
            (diff-pos (send orig-coords :difference-position current-goal-coords))
            (diff-rot (send orig-coords :difference-rotation current-goal-coords :rotation-axis :xy)))
       (cond
         ((and (eps= (norm diff-pos) 0) (eps= (norm diff-rot) 0))
          (setq current-goal-coords nil))
         (t
          (send robot :newcoords (midcoords (min (/ 10 (max (norm diff-pos) 10))
                                                 (/ 0.02 (max (norm diff-rot) 0.02)))
                                            orig-coords current-goal-coords))))
   )) ;; when
   (send-super :robot-interface-simulation-callback)
   )
  (:clear-costmap
   ()
   "Send signal to clear costmap for obstacle avoidance to move_base."
   (let ((srv-name (format nil "~A/clear_costmaps" (send move-base-action :name))))
     (if (ros::wait-for-service srv-name 0)
         (call-empty-service srv-name)))
   t)
  (:change-inflation-range
   (&optional (range 0.2)
    &key (node-name "/move_base_node")
         (costmap-name "local_costmap")
         (inflation-name "inflation"))
   "Changes inflation range of local costmap for obstacle avoidance."
   (let ((srv-name (format nil "~A/~A/~A/set_parameters" node-name costmap-name inflation-name))
         (req (instance dynamic_reconfigure::ReconfigureRequest :init)))
     (send req :config :doubles
           (list (instance dynamic_reconfigure::DoubleParameter :init
                           :name "inflation_radius" :value range)))
     (if (ros::wait-for-service srv-name 0)
         (ros::service-call srv-name req)))
   t)
)

;;;;
(defclass ros-interface
  :super robot-interface
  :slots ())
(defmethod ros-interface
  (:init
   (&rest args)
   (ros::ros-error "please use robot-interface class, ros-interface class will be removed")
   (send-super* :init args))
  )

(defun joint-list->joint_state (jlist &key (position) (effort 0) (velocity 0))
  (let (nlist plist vlist elist)
    (cond
     ((numberp velocity)
      (setq vlist (instantiate float-vector (length jlist)))
      (fill vlist velocity))
     ((float-vector-p velocity)
      (setq vlist (copy-object velocity)))
     )
    (cond
     ((numberp effort)
      (setq elist (instantiate float-vector (length jlist)))
      (fill elist effort))
     ((float-vector-p effort)
      (setq elist (copy-object effort)))
     )
    (cond
     ((numberp position)
      (setq plist (instantiate float-vector (length jlist)))
      (fill plist position))
     ((float-vector-p position)
      (setq plist (copy-object position)))
     )
    (dolist (j jlist)
      (let* ((n (send j :name))
             (nm (if (symbolp n) (symbol-name n) n)))
        (push nm nlist)
        (unless position
          (push (send j :ros-joint-angle) plist))
        ))
    (instance sensor_msgs::JointState :init
              :name (nreverse nlist)
              :position (if position plist
                          (coerce (nreverse plist) float-vector))
              :velocity vlist
              :effort elist)
    ))

(defun apply-joint_state (jointstate robot)
  (let ((cntr 0)
        (ps (send jointstate :position))
        (vl (send jointstate :velocity))
        (ef (send jointstate :effort)))
    (if (/= (length vl) (length (send jointstate :name))) (setq vl nil)) ;; vl is not set
    (if (/= (length ef) (length (send jointstate :name))) (setq ef nil)) ;; ef is not set
    (dolist (jn (send jointstate :name))
      (let ((jk (intern (string-upcase jn) *keyword-package*)))
        (when (find-method robot jk)
          (let ((j (send robot jk)))
            (send j :ros-joint-angle (elt ps cntr))
            (if vl (send j :joint-velocity (elt vl cntr)))
            (if ef (send j :joint-torque (elt ef cntr))))))
      (incf cntr))
    ))

(defun apply-trajectory_point (names trajpoint robot)
  (let ((cntr 0)
        (ps (send trajpoint :positions))
        ;;(vl (send trajpoint :velocities))
        ;;(ef (send trajpoint :accelerations))
        )
    (dolist (jn names)
      (let ((j (send robot (intern (string-upcase jn) *keyword-package*))))
        (send j :ros-joint-angle (elt ps cntr))
        (incf cntr)))
    ))

(defun apply-joint_trajectory (joint-trajectory robot &optional (offset 200.0))
  (let ((names (send joint-trajectory :joint_names))
        (points (send joint-trajectory :points))
        avs tms ptm)
    (dolist (p points)
      (apply-trajectory_point names p robot)
      (push (send robot :angle-vector) avs)
      (cond
       ((eq p (car points))
        (setq ptm (* 1000.0 (send (send p :time_from_start) :to-sec)))
        (push (+ ptm offset) tms))
       (t
        (let ((tm (* 1000.0 (send (send p :time_from_start) :to-sec))))
          (push (- tm ptm) tms)
          (setq ptm tm)))
        ))
    (list (nreverse avs)
          (nreverse tms))
    ))

(defun make-robot-interface-from-name (name &rest args)
  "make a robot model from string: (make-robot-model \"pr2\")"
  (let ((klass (read-from-string (format nil "~A-interface" robot-name))))
    (if (and (boundp klass) (subclassp (eval klass) robot-interface))
        (instance* (eval klass) :init args)
      (error "No such subclass of robot-interface : ~A~%" klass))))

(defun init-robot-from-name (robot-name &rest args)
  "call ${robot}-init function"
  (apply (symbol-function
          (intern (string-upcase (format nil "~A-init" robot-name)))) args)
  (setq *robot* (eval (intern (string-upcase (format nil "*~A*" robot-name))))))


;;
;; navigation-client.l
;;

(defun clear-costmap (&key (node-name "/move_base_node"))
  "reset local costmap and clear unknown grid around robot"
  (if (and (boundp '*ri*) (derivedp *ri* robot-move-base-interface))
      (send *ri* :clear-costmap)
      ;; for backward compatibility
      (let ((srv-name (format nil "/~A/clear_costmaps" node-name)))
        (if (ros::wait-for-service srv-name 0)
            (call-empty-service srv-name))))
  t)

(defun change-inflation-range (&optional (range 0.2)
                                         &key (node-name "/move_base_node")
                                              (costmap-name "local_costmap")
                                              (inflation-name "inflation")
                                         )
  "change inflation range of local costmap"
  (if (and (boundp '*ri*) (derivedp *ri* robot-move-base-interface))
      (send *ri* :change-inflation-range range)
      ;; for backward compatibility
      (let ((srv-name (format nil "~A/~A/~A/set_parameters" node-name costmap-name inflation-name))
            (req (instance dynamic_reconfigure::ReconfigureRequest :init)))
        (send req :config :doubles
              (list (instance dynamic_reconfigure::DoubleParameter :init
                              :name "inflation_radius" :value range)))
        (if (ros::wait-for-service srv-name 0)
            (ros::service-call srv-name req))))
  t)

;;
;; robot-init
;;
(defun robot-init (&optional (robot-name (ros::get-param "/robot/type")))
  "Initialize robot-model and robot-interface instances respectively, such as *pr2* and *ri*.
   Behavior:
     1. Without /robot/type and argument, or no interface file is found, robot instance and interface instance are not created and errors are invoked.
     2. If argument are specified and robot interface file is found, robot instance and interface with given name are created.
     3. If no argument is specified, /robot/type ROSPARAM is set, and robot interface file is found, robot instance and interface with given name are created.
   Typical usage:
     (robot-init) ;; With setting /robot/type ROSPARAM anywhere.
     (robot-init (or (ros::get-param \"/robot/type\") \"pr2\")) ;; If /robot/type ROSPARAM is specified, use it. Otherwise, use \"pr2\" by default.
   Configuring user-defined robot:
     This function searches robot interface file from rospack plugins.
     If user want to use their own robots from robot-init function,
     please write export tag in [user_defined_rospackage]/package.xml as pr2eus/package.xml.
   "
  (let* (;; Check existence ros::rospack-plugins for robot-name-list and interface-file-list.
         ;;    For example, old deb environment before ros::rospack-plugins is committed.
         (robot-name-list
          (if (fboundp 'ros::rospack-plugins)
              (mapcar #'cdr (ros::rospack-plugins "pr2eus" "robot-name"))
            (piped-fork-returns-list "rospack plugins --attrib=robot-name pr2eus | cut -d\\  -f2")))
         (interface-file-list
          (if (fboundp 'ros::rospack-plugins)
              (mapcar #'cdr (ros::rospack-plugins "pr2eus" "interface-file"))
            (piped-fork-returns-list "rospack plugins --attrib=interface-file pr2eus | cut -d\\  -f2")))
         (robot-name-interface-file-list
          (assoc robot-name (mapcar #'(lambda (x y) (list x y))
                                    robot-name-list interface-file-list)
                 :test #'string=)))
    (cond
     ((or (not robot-name-interface-file-list)
          (not (probe-file (cadr robot-name-interface-file-list))))
      (error ";; No such robot interface settings are defined ~A, or interface file not found ~A!!~%" robot-name (cadr robot-name-interface-file-list))
      nil)
     (t
      (require (cadr robot-name-interface-file-list))
      (funcall (eval (read-from-string (format nil "#'~A-init" robot-name))))
      ))))

(provide :robot-interface "robot-interface.l")
