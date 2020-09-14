#!/usr/bin/env roseus
(ros::load-ros-manifest "tabletop_object_detector")
(ros::load-ros-manifest "tabletop_collision_map_processing")
(ros::load-ros-manifest "pr2_template_based_grasping")
(ros::load-ros-manifest "grasp_template_planning")
(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "object_manipulation_msgs")
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")
(ros::roseus-add-msgs "jsk_gui_msgs")

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")

(defun init ()
  (when (not (boundp '*ri*))
    (ros::roseus "template_grasp_eus")
    (pr2)
    (setq *ri* (instance pr2-interface :init))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
  )

(defun check-service-response (srvname &key (wait-time 2))
  (if (and (ros::service-exists srvname)
	   (ros::wait-for-service srvname wait-time))
      t
    (progn (ros::ros-error (format nil "~a service failed" srvname))
	   nil)))

;;originally copied from roseus_tutorials/euslisp/tabletop-object-detector.l
(defun tabletop-detect()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init)) res ret tableply msg)
    (setq res (ros::service-call "tabletop_segmentation" req))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))
    (cond
     ((= (send res :result)
         tabletop_object_detector::TabletopSegmentation::*SUCCESS*)
      (ros::ros-info ";; tabletop detection succeeded ~d" (send res :result))
      (setq ret ;;use make-eus-pointcloud ?
	    (mapcar #'(lambda (p)
			(let ((r (make-eus-pointcloud-from-ros-msg1 p)))
			  (setf (get r :header) (send p :header))
			  r))
		    (send res :clusters)))
      (setq tableply
            (instance polygon :init :vertices (list
                                               (send (send (send (ros::tf-pose->coords (send res :table :pose :pose)) :translate (float-vector 0 0 51) :world) :translate (float-vector (* 1000 (send res :table :x_min)) (* 1000 (send res :table :y_min)) 0)) :worldpos)
                                               (send (send (send (ros::tf-pose->coords (send res :table :pose :pose)) :translate (float-vector 0 0 51) :world) :translate (float-vector (* 1000 (send res :table :x_max)) (* 1000 (send res :table :y_min)) 0)) :worldpos)
                                               (send (send (send (ros::tf-pose->coords (send res :table :pose :pose)) :translate (float-vector 0 0 51) :world) :translate (float-vector (* 1000 (send res :table :x_max)) (* 1000 (send res :table :y_max)) 0)) :worldpos)
                                               (send (send (send (ros::tf-pose->coords (send res :table :pose :pose)) :translate (float-vector 0 0 51) :world) :translate (float-vector (* 1000 (send res :table :x_min)) (* 1000 (send res :table :y_max)) 0)) :worldpos))))
      )
     (t
      (ros::ros-warn ";; tabletop detection failed ~d" (send res :result))
      (setq ret nil)
      ))
    (list ret tableply)
    ))

(defun make-eus-pointcloud (pc-list)
  (mapcar #'(lambda (p)
	      (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
		(setf (get r :header) (send p :header))
		r))
	  pc-list))

;;; Function calls tabletop object detection service. Results of
;;; object detection service (segmentation) and collision map (for
;;; grasp and path planning) are placed in global variables *od-res*
;;; and *cp-res* respectively.
(defun template-grasp-get-data()
  (let* ((odsrv-name "/object_detection")
	 (cpsrv-name "/tabletop_collision_map_processing/tabletop_collision_map_processing")
	 (od-req (instance tabletop_object_detector::TabletopDetectionRequest :init))
	 (cp-req (instance tabletop_collision_map_processing::TabletopCollisionMapProcessingRequest :init))
	 od-res cp-res
	 )
    (send od-req :return_clusters t)
    (send od-req :return_models t)

    (if (check-service-response odsrv-name)
	(setq od-res (ros::service-call odsrv-name od-req))
      (return-from template-grasp-get-data nil))

    (unless (= (send od-res :detection :result) tabletop_object_detector::TabletopDetectionResult::*SUCCESS*)
      (ros::ros-error (format nil "Tabletop detection returned error code ~A" (send od-res :detection :result)))
      (return-from template-grasp-get-data nil))

    (when (and (null (send od-res :detection :clusters)) (null (send od-res :detection :models)))
      (ros::ros-error "The tabletop detector detected the table, but found no objects")
      (return-from template-grasp-get-data nil))

    (ros::ros-info "Calling collision map processing")
    (send cp-req :detection_result (send od-res :detection))
    (send cp-req :reset_collision_models t)
    (send cp-req :reset_attached_models t)
    (send cp-req :desired_frame "/base_link")
    (if (check-service-response cpsrv-name)
	(setq cp-res (ros::service-call cpsrv-name cp-req))
      (return-from template-grasp-get-data nil))

    (unless (send cp-res :graspable_objects)
      (ros::ros-error "Collision map processing returned no graspable objects")
      (return-from template-grasp-get-data nil))

    (setq *od-res* od-res)
    (setq *cp-res* cp-res)
    ))

;; no more support on arm_planning
#|
(defun template-grasp-get-object
  (grasp-target-num &key (arm :rarm) (od-res *od-res*) (cp-res *cp-res*))
  (let* ((grasp-target (elt (send cp-res :graspable_objects) grasp-target-num))
	 (co-name (elt (send cp-res :collision_object_names) grasp-target-num))
	 (gpsrv-name "/pr2_template_grasp_planner")
	 (gfsrv-name "/pr2_template_grasp_planner_feedback")
	 (gp-req (instance object_manipulation_msgs::GraspPlanningRequest :init))
	 (gf-req (instance pr2_template_based_grasping::PlanningFeedbackRequest :init))
	 gp-res gf-res
	 (picka-name "/object_manipulator/object_manipulator_pickup")
	 (heada-name "/head_traj_controller/point_head_action")
	 (pickup-client (instance ros::simple-action-client :init picka-name object_manipulation_msgs::PickupAction))
	 (pickup-goal (instance object_manipulation_msgs::PickupActionGoal :init))
	 (place-goal (instance object_manipulation_msgs::PlaceActionGoal :init))
	 pickup-res
	 (head-client (instance ros::simple-action-client :init heada-name pr2_controllers_msgs::PointHeadAction))
	 (head-goal (instance pr2_controllers_msgs::PointHeadActionGoal :init))
	 (direction (instance geometry_msgs::Vector3Stamped :init))
	 (pickup-location (instance geometry_msgs::PoseStamped :init))
	 )
    (ros::ros-info "Calling template grasp planner")
    (send gp-req :target :cluster (elt (send od-res :detection :clusters) grasp-target-num))
    (send gp-req :target :region :roi_box_pose (send od-res :detection :table :pose))
    (if (check-service-response gpsrv-name)
	(setq gp-res (ros::service-call gpsrv-name gp-req))
      (return-from template-grasp-get-object nil))

    (send pickup-goal :goal :desired_grasps (send gp-res :grasps))
    (send pickup-goal :goal :allow_gripper_support_collision t)

    (send pickup-goal :goal :target grasp-target)
    (send pickup-goal :goal :collision_object_name co-name)
    (send pickup-goal :goal :collision_support_surface_name (send cp-res :collision_support_surface_name))
    (send pickup-goal :goal :arm_name
	  (if (eq arm :rarm) "right_arm" "left_arm"))
    (send direction :header :stamp (ros::time-now))
    (send direction :header :frame_id "/base_link")
    (send direction :vector :x 0)
    (send direction :vector :y 0)
    (send direction :vector :z 1)
    (send pickup-goal :goal :lift :direction direction)
    (send pickup-goal :goal :lift :desired_distance 0.1)
    (send pickup-goal :goal :lift :min_distance 0.05)
    (send pickup-goal :goal :use_reactive_execution nil)
    (send pickup-goal :goal :use_reactive_lift nil)
    (send pickup-client :send-goal pickup-goal)
    (send pickup-client :wait-for-result :timeout 10)
    (setq pickup-res (send pickup-client :get-result))
    (unless (= (send pickup-client :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
      (ros::ros-error (format nil "The pickup action has failed with result code ~a" (send pickup-res :manipulation_result :value)))
      (return-from template-grasp-get-object nil))

    (send gf-req :action pr2_template_based_grasping::PlanningFeedback::*DONT_UPGRADE_LIB*)
    (send gf-req :feedback pickup-res)
    (ros::ros-info "Calling template grasp planning feedback...")
    (if (check-service-response gfsrv-name)
    	(setq gf-res (ros::service-call gfsrv-name gf-req))
      (return-from template-grasp-get-object nil))

    (if (/= 0 (length (send grasp-target :potential_models)))
    	(setq pickup-location (send (car (send grasp-target :potential_models)) :pose))
      (progn
    	(send pickup-location :header (send grasp-target :cluster :header))
    	(send pickup-location :pose :position :x (send (car (send grasp-target :cluster :points)) :x))
    	(send pickup-location :pose :position :y (send (car (send grasp-target :cluster :points)) :y))
    	(send pickup-location :pose :position :z (send (car (send grasp-target :cluster :points)) :z))
    	(send pickup-location :pose :orientation :w 1)))

    (send head-goal :target :point (send pickup-location :pose :position))
    (ros::ros-info "Calling the place action")
    ))
|#

;;; Given index of object to be picked, pickup plan is created and
;;; executed. Plan consist of deciding correct grasp for the object
;;; and then creating a path for the arm using OpenRave.
(defun template-grasp-get-object-openrave
  (grasp-target-num &key (arm :rarm) (show-all-grasps nil)
                    (od-res *od-res*) (cp-res *cp-res*) (use-select t))
  (let* ((grasp-target (elt (send cp-res :graspable_objects) grasp-target-num))
         (gpsrv-name "/pr2_template_grasp_planner")
         (gp-req (instance object_manipulation_msgs::GraspPlanningRequest :init))
         gp-res max-grasps (counter 0))

    (ros::ros-info "Calling template grasp planner")
    (send gp-req :target :cluster (elt (send od-res :detection :clusters) grasp-target-num))
    (send gp-req :target :region :roi_box_pose (send od-res :detection :table :pose))

    ;; Call Grasp planner service and handle possible errors.
    (if (check-service-response gpsrv-name)
        (setq gp-res (ros::service-call gpsrv-name gp-req)))
    (unless (= (send gp-res :error_code :value)
               object_manipulation_msgs::GraspPlanningErrorCode::*SUCCESS*)
      (ros::ros-error (format nil "Template grasp planner returned error code ~A"
                              (send gp-res :error_code :value)))
      (return-from template-grasp-get-object-openrave nil))

    ;; Transform and translate coordinates of the grasps to be
    ;; compatible with OpenRave.
    (setq max-grasps (length (send gp-res :grasps)))
    (let ((world-z-offset (float-vector 0 0 51))
          (local-endpos-offset
           (send (send *pr2* (case arm (:rarm :r_wrist_roll_link) (t :l_wrist_roll_link)))
                 :transformation (send *pr2* arm :end-coords))))
      (setq grasp-cds-lst
            (mapcar #'(lambda (gp)
                        (let ((cds (ros::tf-pose->coords (send gp :grasp_pose))))
                          (send cds :translate world-z-offset :world) ;; /base_link -> /base_footprint
                          (send cds :transform local-endpos-offset) ;; wrist_roll_link -> end-coords
                          cds)) (send gp-res :grasps))))
    ;; show grasps
    (when show-all-grasps
      (dolist (grasp-cds grasp-cds-lst)
        (unix::usleep (* 30 1000))
        (send *ri* :show-goal-hand-coords grasp-cds arm)))

    (setq counter 0)
    (dolist (grasp-cds grasp-cds-lst)
      (let (orres
            (pre-grasp-cds;; offset
             (let ((c (send grasp-cds :copy-worldcoords)))
               (send c :translate (float-vector
                                   (* (+
                                       (send (elt (send gp-res :grasps) counter)
                                             :desired_approach_distance)
                                       (send (elt (send gp-res :grasps) counter)
                                             :min_approach_distance))
                                      -0.5 1000)
                                   0 0)) c)) ;; approach offset
            hand-coords-result
            hand-coords)
        (print (list 'handdirection (v. (send (send grasp-cds :worldcoords) :x-axis)
                                        (float-vector 0 0 -1))))
        (print (list 'zpos (elt (send grasp-cds :pos) 2)))
        (print (list 'tablezpos (* (send od-res :detection :table
                                         :pose :pose :position :z) 1000)))

        (setq hand-coords-result (send *ri* :show-goal-hand-coords grasp-cds arm))
        ;; temporary
        (setq hand-coords (cadr hand-coords-result))
        (visualize-object-image-from-coords
         (list (elt hand-coords 4) (elt hand-coords 2)
               (elt hand-coords 0) (elt hand-coords 1) (elt hand-coords 3))
         :counter 601 :lifetime 10 :ns "hand")
        (when (and
               ;;hand direction should be downword

               (>= (v. (send (send grasp-cds :worldcoords) :x-axis)
                       (float-vector 0 0 -1)) 0)

               ;;higher than table

               (> (elt (send grasp-cds :pos) 2)
                  (+ (* (send od-res :detection :table
                              :pose :pose :position :z) 1000) 30))

               ;; select correct hand pos from tablet
               ;; (if use-select (y-or-n-p) t)
               (if use-select (y-or-n-from-tablet) t)
               ;; solve motion-plan at OpenRAVE
               (setq orres (send *ri* :move-end-coords-plan pre-grasp-cds
                                 :move-arm arm :use-torso t :send-trajectory nil))
               )
          (ros::ros-info "orres:~A" orres)

          (let (eusres)
            (send *pr2* :angle-vector (car (last (elt orres 0))))
            (setq eusres
                  (send *pr2* arm :inverse-kinematics (send grasp-cds :translate (float-vector 50 0 0))
                        :rotation-axis t))

            (ros::ros-info "eusres: ~A" eusres)
            (when eusres
              (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
              (send *ri* :angle-vector-sequence (elt orres 0) (elt orres 1))
              (send *ri* :wait-interpolation)
              (send *ri* :update-robot-state)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :pre_grasp_posture :position) 0))
                    :wait t :effort 20)
              (send *ri* :angle-vector eusres 2000)
              (send *ri* :wait-interpolation)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :grasp_posture :position) 0))
                    :wait t :effort 20)
              (return t))))
        (setq counter (+ counter 1))))
    ))

(defun template-grasp-get-object-eus
  (grasp-target-num &key (arm :rarm) (show-all-grasps nil)
                    (od-res *od-res*) (cp-res *cp-res*) (use-select t))
  (let* ((grasp-target (elt (send cp-res :graspable_objects) grasp-target-num))
         (gpsrv-name "/pr2_template_grasp_planner")
         (gp-req (instance object_manipulation_msgs::GraspPlanningRequest :init))
         gp-res max-grasps (counter 0))

    (ros::ros-info "Calling template grasp planner")
    (send gp-req :target :cluster (elt (send od-res :detection :clusters) grasp-target-num))
    (send gp-req :target :region :roi_box_pose (send od-res :detection :table :pose))

    ;; Call Grasp planner service and handle possible errors.
    (if (check-service-response gpsrv-name)
        (setq gp-res (ros::service-call gpsrv-name gp-req)))
    (unless (= (send gp-res :error_code :value)
               object_manipulation_msgs::GraspPlanningErrorCode::*SUCCESS*)
      (ros::ros-error (format nil "Template grasp planner returned error code ~A"
                              (send gp-res :error_code :value)))
      (return-from template-grasp-get-object-openrave nil))

    ;; Transform and translate coordinates of the grasps to be
    ;; compatible with OpenRave.
    (setq max-grasps (length (send gp-res :grasps)))
    (let ((world-z-offset (float-vector 0 0 51))
          (local-endpos-offset
           (send (send *pr2* (case arm (:rarm :r_wrist_roll_link) (t :l_wrist_roll_link)))
                 :transformation (send *pr2* arm :end-coords))))
      (setq grasp-cds-lst
            (mapcar #'(lambda (gp)
                        (let ((cds (ros::tf-pose->coords (send gp :grasp_pose))))
                          (send cds :translate world-z-offset :world) ;; /base_link -> /base_footprint
                          (send cds :transform local-endpos-offset) ;; wrist_roll_link -> end-coords
                          cds)) (send gp-res :grasps))))
    ;; show grasps
    (when show-all-grasps
      (dolist (grasp-cds grasp-cds-lst)
        (unix::usleep (* 30 1000))
        (send *ri* :show-goal-hand-coords grasp-cds arm)))

    (setq counter 0)
    (dolist (grasp-cds grasp-cds-lst)
      (let (orres
            (pre-grasp-cds;; offset
             (let ((c (send grasp-cds :copy-worldcoords)))
               (send c :translate (float-vector
                                   (* (+
                                       (send (elt (send gp-res :grasps) counter)
                                             :desired_approach_distance)
                                       (send (elt (send gp-res :grasps) counter)
                                             :min_approach_distance))
                                      -0.5 1000)
                                   0 0)) c)) ;; approach offset
            hand-coords-result
            hand-coords)

        (setq hand-coords-result (send *ri* :show-goal-hand-coords grasp-cds arm))
        ;; temporary
        (setq hand-coords (cadr hand-coords-result))
        (visualize-object-image-from-coords
         (list (elt hand-coords 4) (elt hand-coords 2)
               (elt hand-coords 0) (elt hand-coords 1) (elt hand-coords 3))
         :counter 601 :lifetime 10 :ns "hand")
        (when (and
               ;; hand direction should be downword

               ;; (>= (v. (send (send grasp-cds :worldcoords) :x-axis)
               ;;         (float-vector 0 0 -1)) 0)

               ;; higher than table

               ;; (> (elt (send grasp-cds :pos) 2)
               ;;    (+ (* (send od-res :detection :table
               ;;                :pose :pose :position :z) 1000) 30))

               ;; select correct hand pos from tablet
               ;; (if use-select (y-or-n-p) t)
               (if use-select (y-or-n-from-tablet) t)
               ;; solve motion-plan at OpenRAVE
               ;; (setq orres (send *ri* :move-end-coords-plan pre-grasp-cds
               ;;                   :move-arm arm :use-torso t :send-trajectory nil))
               (setq orres
                     (send *pr2* arm :inverse-kinematics pre-grasp-cds
                           :rotation-axis t))
               )
          (ros::ros-info "orres:~A" orres)

          (let (eusres)
            ;; (send *pr2* :angle-vector (car (last (elt orres 0))))
            (setq eusres
                  (send *pr2* arm :inverse-kinematics grasp-cds :pos)
                        :rotation-axis t)
            (ros::ros-info "eusres: ~A" eusres)

            (print orres)
            (print eusres)

            (when eusres
              (send *pr2* :angle-vector orres)
              ;;(send *ri* :wait-interpolation)
              (send *ri* :angle-vector (send *pr2* :angle-vector))
              (unix::sleep 4)
             ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
             ;; (send *ri* :angle-vector-sequence (elt orres 0) (elt orres 1))
             ;; (send *ri* :wait-interpolation)
              (send *ri* :update-robot-state)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :pre_grasp_posture :position) 0))
                    :wait t :effort 20)
              (send *ri* :angle-vector eusres 2000)
              (send *ri* :wait-interpolation)
              (send *ri* :move-gripper arm
                    (* 2 (elt (send (car (send gp-res :grasps))
                                    :grasp_posture :position) 0))
                    :wait t :effort 20)
              (return t))))
        (setq counter (+ counter 1))))
    ))

(defun calc-object-pos (res)
  (let (calc-pos)
    (setq calc-pos (mapcar
		    #'(lambda(res)
			(let ((cl (send res :cluster)))
			  (scale (/ 1.0 (length (send cl :points)))
				 (reduce #'v+
					 (mapcar
					  #'(lambda(pt)(ros::tf-point->pos pt))
					  (send cl :points))))))
		    res))
    calc-pos))

;;(setq hoge (car (send *od-res* :detection :clusters)))
(defun calc-pointcloud-pos (res)
  (let (calc-pos)
    (setq calc-pos (mapcar
		    #'(lambda(cl)
			(scale (/ 1.0 (length (send cl :points)))
			       (reduce #'v+
				       (mapcar
					#'(lambda(pt)(ros::tf-point->pos pt))
					(send cl :points)))))
			res))
    calc-pos))

(defun find-nearest (target-point points-lst)
  (unless (listp points-lst) (ros::ros-error "second arg must be list")
	  (return-from find-nearest nil))
  (let ((point-num 0) (min-num 0)
	(min-distance (distance target-point (car points-lst))))
    (dolist (point points-lst)
      (let ((current-distance (distance target-point point)))
	(if (> min-distance current-distance)
	    (progn (setq min-distance current-distance)
		   (setq min-num point-num)))
	(incf point-num)))
    min-num))

(defun tablet-setup ()
  (ros::load-ros-manifest "jsk_smart_gui")
  (setq *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")
  ;; for is-old-msg and screenpoinot-srvcall
  (load "package://jsk_smart_gui/src/utils.l")

  (subscribe-tablet-command-default)
  (subscribe-tablet-command-select)
  (ros::advertise "image_marker" image_view2::ImageMarker2 100)
  (ros::advertise "tablet_marker" visualization_msgs::Marker 10)
  (ros::advertise "tablet_marker_array" visualization_msgs::MarkerArray 10)
  (ros::advertise "tabletop_detection_marker_array" visualization_msgs::MarkerArray 10)

  (setq *time-count* (ros::time-now))
  (setq *current-camera* "/openni/rgb") ;; not needed anymore?
  (setq *has-reset-collider* nil)
  )

(defun subscribe-tablet-command-default ()
  (ros::subscribe "/Tablet/Command" jsk_gui_msgs::Tablet #'tablet-command-cb))

(defun unsubscribe-tablet-command-default ()
  (ros::unsubscribe "/Tablet/Command"))

(defun subscribe-tablet-command-select ()
  (ros::subscribe "/Tablet/Select" roseus::StringStamped #'tablet-select-cb))

(defun unsubscribe-tablet-command-select ()
  (ros::unsubscribe "/Tablet/Select")
  )

(defun tablet-select-cb (msg)
  (if (is-old-msg msg) (return-from tablet-select-cb nil))
  (ros::ros-info "tablet-select-cb called")
  (let ((taskname (read-from-string (send msg :data))))
    (case taskname
      ('ResultYes
       (setq *select-result* t)
       )
      ('ResultNo
       (setq *select-result* nil)
       ))
    (setq *select-flag* t)
    ))

(defun tablet-command-cb (msg)
  (if (is-old-msg msg) (return-from tablet-command-cb nil))
  (let* ((touches (send msg :touches))
	 (taskname (read-from-string (send msg :action :task_name)))
to	 (arm-id-raw (send msg :action :arm_id))
	 (arm-id (if (or (zerop arm-id-raw) (equal arm-id-raw jsk_gui_msgs::Action::*RARMID*)) ':rarm ':larm)))
    (ros::ros-info "tablet-command-cb called: arm-id~A" arm-id)
    (case taskname
      ('StartDetect
       (ros::ros-info "StartDetect called")
       )
      ('ResetCollider
       (ros::ros-info "ResetCollider called")
       (call-empty-service "/collider_node/reset")
       (setq *has-reset-collider* t)
       )
      ('PassToHumanOnce
       (ros::ros-info "PassToHumanOnce called")
       (PassToHumanExecute arm-id :wait-shock 10)
       )
      ('ScanObject
       (ros::ros-info "ScanObject called")
       (visualize-object-image (tabletop-detect))
       )
      ('PickObjectSelected
       (ros::ros-info "PickObjectSelected called")
       (PickObjectExecute (send msg :action :touch_x) (send msg :action :touch_y) arm-id)
       )
      ('PlaceObjectSelected
       (ros::ros-info "PlaceObjectSelected called")
       (PlaceObjectExecute arm-id)
       )
      )))

;; copied from jsk_2011_07_pr2_semantic/euslisp/actions.l
(defun wait-for-hand-impact (arm &key (timeout 30))
  (let* ((action-name (format nil "/~c_gripper_sensor_controller/event_detector" (if (eq arm :larm) #\l #\r)))
         (client (instance ros::simple-action-client :init action-name pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
         (goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init)))
    (unless (send client :wait-for-server 5)
      (return-from wait-for-hand-impact nil))
    (send goal :header :stamp (ros::time-now))
    (send goal :goal_id :stamp (ros::time-now))
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
    (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_ACC*)
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*SLIP*)
    (send goal :goal :command :slip_trigger_magnitude 0.02)
    (send goal :goal :command :acceleration_trigger_magnitude 3.0) ;; m/s^2
    (send client :send-goal goal)
    (ros::ros-info "wait for touching robot hand")
    (send client :wait-for-result :timeout timeout))
  )

(defun PassToHumanExecute (arm &key (wait-shock nil))
  (ros::spin-once)
  (let* ((av (send *ri* :state :potentio-vector))
	(tuckarm (check-tuckarm-pose))
	(isfreearm (eq arm tuckarm))
	;; this is for :larm
	(avs (list #f(12 0 64 70 -122 50 -115 160 -4 74 -105 -90 70 -5 20 2 15)
		   #f(12 6 9 106 -77 35 -124 -128 -4 75 -104 -89 70 0 20 3 30)
		   #f(12 13 21 62 -105 -117 -66 -71 -4 74 -104 -89 70 -5 20 4 40)
		   #f(12 9 24 50 -94 -158 -70 39 -4 74 -104 -89 70 -5 20 5 30)))
	(tms (make-list (length avs) :initial-element 1000))
	(l-r-reverse #f(1  -1 1 -1 1 -1 1 -1  -1 1 -1 1 -1 1 -1  -1 1)))
    ;;
    (if (eq arm :rarm)
	(setq avs
	      (mapcar #'(lambda(av)
			  (map float-vector #'*
			       (concatenate float-vector
					    (subseq av 0 1) (subseq av 8 15)
					    (subseq av 1 8) (subseq av 15 17))
			       l-r-reverse))
			  avs)))
    ;;
    (unless isfreearm
      (pr2-reset-pose)
      (setq avs (subseq avs (- (length avs) 2))
	    tms (subseq tms (- (length avs) 2))
	    av (send *ri* :state :potentio-vector))
      (setq tuckarm arm))
    ;;
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    ;;
    (if (and wait-shock (not (numberp wait-shock))) ;; if wait-shock = t
	(setq wait-shock 10))
    (if wait-shock
	(progn (wait-for-hand-impact arm :timeout wait-shock)
	       (ros::ros-info "return from gripper sensor event")
	       (send *ri* :move-gripper arm 0.08 :wait t))
      (progn
	(send *ri* :move-gripper arm 0.08 :wait t)
	(unix::sleep 3)))
    ;;
    (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list av)) tms)
    (send *ri* :move-gripper arm 0.00 :wait nil)
    (send *ri* :wait-interpolation)
    ;;
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (pr2-tuckarm-pose tuckarm)
    ))

(defun stop-visualize-object-image ()
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :id -1)
    (send mrk :type image_view2::ImageMarker2::*LINE_LIST3D*)
    (send mrk :action image_view2::ImageMarker2::*REMOVE*)
    (ros::publish "image_marker" mrk)))

(defun stop-visualize-object-rviz
  (&key (idn 100) (ns "") (topic-name "marker-array"))
  (dotimes (i idn) (remove-marker i :ns ns :topic-name topic-name))
  ) ;; remove-marker in pr2eus-openrave.l

(defun visualize-object-rviz
  (pc-list &key (counter 0) (ns nil) (lifetime 10)
	   (color #f(0 0 1)))
  (let ((points-list pc-list) (cntr counter)
	(mrk-array (instance visualization_msgs::MarkerArray :init))
	mrk-list)
    (dolist (points points-list)
      (let* ((c (send points :centroid))
	     (b (send points :box))
	     (cb
	      (apply #'make-cube (coerce (send b :diagonal) cons)))
	     (header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "/base_link")))
	(send cb :translate c)
	(send cb :worldcoords)
	(push (wireframe->marker-msg cb header :id (incf cntr) :lifetime lifetime :color color :ns ns) mrk-list)
	))
    (send mrk-array :markers mrk-list)
    (ros::publish "tabletop_detection_marker_array" mrk-array)
    ))

(defun visualize-object-image-from-coords
  (cds-list &key (counter 0) (ns nil) (lifetime 10)
	    (outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))))
  (let* ((mrk (instance image_view2::ImageMarker2 :init))
	 point-list)
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*LINE_STRIP3D*)
    (send mrk :points3D :header :frame_id "/base_link")
;;    (send mrk :points3D :header :frame_id "base_footprint")
    (send mrk :lifetime (ros::Time lifetime))
    (send mrk :id counter)
    (dolist (cds cds-list)
      ;;(ros::ros-warn "cds: ~A" (send cds :pos))
      (push (ros::pos->tf-point (send cds :worldpos)) point-list))
    (send mrk :points3D :points point-list)
    (send mrk :outline_colors outline-colors)
    (if ns (send mrk :ns ns))
    (ros::publish "image_marker" mrk)
  ))

(defun visualize-object-image-from-coords2
  (cds-list &key (counter 0) (ns nil) (lifetime 10)
	    (outline-colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))))
  (let* ((mrk (instance image_view2::ImageMarker2 :init))
	 point-list)
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*LINE_STRIP3D*)
    (send mrk :points3D :header :frame_id "/base_link")
;;    (send mrk :points3D :header :frame_id "base_footprint")
    (send mrk :lifetime (ros::Time lifetime))
    (send mrk :id counter)
    (dolist (cds cds-list)
      ;;(ros::ros-warn "cds: ~A" (send cds :pos))
      (push (ros::pos->tf-point (send cds :worldpos)) point-list))
    (send mrk :points3D :points point-list)
    (send mrk :outline_colors outline-colors)
    (if ns (send mrk :ns ns))
    (ros::publish "image_marker" mrk)
  ))

(defun visualize-object-image
  (pc-list &key (counter 0) (use-text t) (ns nil)
	   (text nil) (lifetime 10) (text-lifetime 40)
	   (outline-colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0)))
	   )
  (let ((points-list pc-list) (cntr counter) (obj-cntr 0))
    (when points-list
      (dolist (points points-list)
	(let* ((mrk (instance image_view2::ImageMarker2 :init))
	       (text-mrk (instance image_view2::ImageMarker2 :init))
	       poly-vertex-list
	       (c (send points :centroid))
	       (b (send points :box))
	       (cb
		(apply #'make-cube (coerce (send b :diagonal) cons))))
	  (send cb :translate c)
	  (send cb :worldcoords)

	  (send mrk :header :stamp (ros::time-now))
	  ;;(send mrk :action image_view2::ImageMarker2::*ADD*)
	  (send mrk :type image_view2::ImageMarker2::*LINE_LIST3D*)
	  (send mrk :points3D :header :frame_id "/base_link")
	  (send mrk :lifetime (ros::Time lifetime))
	  (send mrk :id cntr)
	  (dolist (poly-vertex-list-eus (mapcan #'(lambda(eds) (send eds :vertices)) (send cb :edges)))
	    (push (ros::pos->tf-point poly-vertex-list-eus) poly-vertex-list))
	  (send mrk :points3D :points poly-vertex-list)
	  (send mrk :outline_colors outline-colors)
	  (if ns (send mrk :ns ns))
	  (ros::publish "image_marker" mrk)
	  (incf cntr)
	  (when use-text
	    (send text-mrk :header :stamp (ros::time-now))
	    (send text-mrk :type image_view2::ImageMarker2::*TEXT3D*)
	    (send text-mrk :position3D :header :frame_id "/base_link")
	    (send text-mrk :position3D :point (ros::pos->tf-point c))
	    (send text-mrk :scale 1.0)
	    (send text-mrk :lifetime (ros::Time text-lifetime))
	    (send text-mrk :id cntr)
	    (send text-mrk :text (if text text (format nil "OBJ~A" obj-cntr)))
	    (if ns (send text-mrk :ns ns))
	    (ros::publish "image_marker" text-mrk)
	    (incf cntr) (incf obj-cntr))
	  )))))

(defun is-current-tuckarm (arm)
  (let ((current-arm (check-tuckarm-pose :thre 40)))
    (if (eq current-arm arm) t nil)))

(defun pr2-tuckarm-pose (&rest args)
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (free-arm (or (car args) current-arm :larm))
		 (msec 500) (use-time t))
    (ros::ros-warn "this is not formal tuckarm-pose!!")
    (when (not (eq current-arm free-arm))
	  (progn
		(setq msec 2000)
		(send *pr2* :larm :angle-vector #f( 25 0 0 -121 0 -6 0))
		(send *pr2* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))
		(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
		(send *ri* :wait-interpolation)))
    (if (eq free-arm :larm)
        (progn
		  (send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-larm-free*))
		  (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec))
	  (progn
		(send *pr2* :larm :angle-vector (car *pr2-tuckarm-pose-rarm-free*))
		(send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec)))
    (unless use-time (print "debug03"))
    (if use-time (unix::sleep 2)
      (send *ri* :wait-interpolation))
    (unless use-time (print "debug04"))
    (if (eq free-arm :larm)
        (progn
		  (send *pr2* :larm :angle-vector (car *pr2-tuckarm-pose-larm-free*))
		  (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
		(send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-rarm-free*))
		(send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec)))
    (unless use-time (print "debug05"))
    (if use-time (unix::sleep 2)
      (send *ri* :wait-interpolation))
    (unless use-time (print "debug06"))
    ))

(defun PlaceObjectExecute (arm)
  ;; down
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* arm :move-end-pos #f(0 0 -100) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;; close
  (send *ri* :stop-grasp)
 )

;;; This function executes the whole process of picking up the
;;; object. Given arguments are X and Y coordinates of tablet screen
;;; and possible openRave usage boolean. XY-value are matched to
;;; XYZ-point in real world and the closest object from there is
;;; picked up with the correct grasp. Grasp selection is done interactively in tablet.
;;(defun PickObjectExecute (x y arm &key (use-openrave nil))
(defun PickObjectExecute (x y arm &key (use-openrave t))
  (let* ((c (screenpoint-srvcall x y)) cpos target-num) ; c = XYZ point in real world
    (if (null c) (return-from PickObjectExecute nil))
    (setq cpos (send (send c :copy-worldcoords) :pos))
    (if use-openrave
	(progn (ros::ros-info "pr2-tuckarm-pose ~A" arm)
	       (unless (is-current-tuckarm arm)
		 (setq *has-reset-collider* nil))
	       ;;(pr2-tuckarm-pose arm)
               )
      ;;(reset-pose)
;;      (pr2-tuckarm-pose arm)
      )
    (if *has-reset-collider*
	(ros::ros-warn "already called /collider_node/reset")
      (progn
	(ros::ros-warn "calling /collider_node/reset")
	;;(call-empty-service "/collider_node/reset");temporary
        ))
    (setq *has-reset-collider* nil)
    (ros::set-param "stop_tabletop" 1)
    (template-grasp-get-data) ;; get *cp-res*, *od-res*
    (unless (boundp '*od-res*) (return-from PickObjectExecute nil))
    ;; (visualize-object-image (make-eus-pointcloud (send *od-res* :detection :clusters))) this is not needed if (realtime-tabletop) is called in background
    (unless (boundp '*cp-res*) (return-from PickObjectExecute nil))
    (setq target-num (find-nearest cpos (calc-object-pos (send *cp-res* :graspable_objects))))
    (ros::ros-info "target: ~A" target-num)
    ;; for demo
    ;; (unix::sleep 5)
    (stop-visualize-object-image)
    (stop-visualize-object-rviz :ns "hand_traj" :topic-name "openrave_marker_array")
    (stop-visualize-object-rviz :ns "tabletop" :topic-name "tabletop_detection_marker_array")

    (unix::usleep (* 100 1000))
    (visualize-object-image (make-eus-pointcloud (list (elt (send *od-res* :detection :clusters) target-num))) :counter 201 :text "Target" :lifetime 20)
    (when (numberp target-num)
      (ros::ros-info "arm: ~A" arm)
      (if use-openrave
	  (template-grasp-get-object-openrave target-num :arm arm :show-all-grasps t)
	(template-grasp-get-object-eus target-num :arm arm)))

    (stop-visualize-object-rviz :ns "hand_traj" :topic-name "openrave_marker_array")
    ;; up
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* arm :move-end-pos #f(0 0 100) :world :rotation-axis nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector))

    (ros::set-param "stop_tabletop" 0)
    ))

(defun y-or-n-from-tablet ()
;;  (unsubscribe-tablet-command-default)
  (unix::usleep (* 200 1000))
;;  (subscribe-tablet-command-select)
  (unix::usleep (* 200 1000))
  (setq *select-flag* nil *select-result* nil)
  (while (not *select-flag*)
    (ros::spin-once)
    (ros::sleep))
  (setq *select-flag* nil)
;;  (unsubscribe-tablet-command-select)
  (unix::usleep (* 200 1000))
;  (subscribe-tablet-command-default)
  (unix::usleep (* 200 1000))
  *select-result*)


(defun reset-pose () ;; only used for arm_planning
  (send *pr2* :angle-vector #f(11.7223 122.335 -20.2339 24.6296 -55.5484 48.3199 -114.592 -176.87 -122.338 -19.5301 -5.80601 -75.6798 -72.7564 -114.592 -155.57 -0.332237 46.746))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun get-data-demo()
  (call-empty-service "/collider_node/reset")
  (template-grasp-get-data))

(defun get-object-demo(target-num arm)
  (template-grasp-get-object-openrave target-num :arm arm))

(defun start-detect-demo ()
  (init)
  (tablet-setup)
  (ros::ros-info "change inflation range to 0.15")
  (change-inflation-range 0.15)
  (ros::ros-info "start demo")
  (do-until-key
    (ros::spin-once)
    (ros::sleep)))

;;(init)
;;(start-detect-demo)

(warn "
(start-detect-demo)
")