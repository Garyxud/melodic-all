;; simulation
;; ROBOT=sim roslaunch pr2eus_tutorials pr2_gazebo_interactive.launch
;; roseus arm-navigation-sample.l
;;
;; real robot
;; roslaunch pr2jsk_arm_navigation_with_torso pr2jsk_arm_navigation.launch
;; roseus arm-navigation-sample.l
;;
(load "package://pr2eus_tutorials/euslisp/pr2main.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l");;
(load "package://pr2eus_armnavigation/pr2eus-arm-navigation.l")
(load "package://pr2eus_armnavigation/collision-object-publisher.l")

;; for using torso setting
(defclass pr2torso_arm_planning_environment
  :super arm_planning_environment
  :slots ())
(defmethod pr2torso_arm_planning_environment
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :action-name "/move_right_arm")
               (cons :target-link-name "r_wrist_roll_link")
               (cons :ik-service-name "/pr2_right_arm_kinematics/get_constraint_aware_ik")
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :action-name "/move_left_arm")
               (cons :target-link-name "l_wrist_roll_link")
               (cons :ik-service-name "/pr2_left_arm_kinematics/get_constraint_aware_ik")
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rarm-torso ;; add for torso
               (cons :group-name "right_torso")
               (cons :action-name "/move_right_torso")
               (cons :target-link-name "r_wrist_roll_link")
               (cons :ik-service-name "/pr2_right_torso_kinematics/get_constraint_aware_ik")
               (cons :joint-list (append (send robot :rarm :joint-list) (send robot :torso :joint-list)))
               )
         (list :larm-torso ;; add for torso
               (cons :group-name "left_torso")
               (cons :action-name "/move_left_torso")
               (cons :target-link-name "l_wrist_roll_link")
               (cons :ik-service-name "/pr2_left_torso_kinematics/get_constraint_aware_ik")
               (cons :joint-list (append (send robot :larm :joint-list) (send robot :torso :joint-list)))
               )
         ))
  )
(setq *collision-server* (instance collision-object-publisher :init))

(send *plan-env* :robot :angle-vector (send *ri* :robot :angle-vector))
(send *plan-env-torso* :robot :angle-vector (send *ri* :robot :angle-vector))

(warn ";;
;; (demo-init)
;; (demo-ik)
;; (demo-planning)
;;
")

(defun demo-init ()
  (pr2-init)
  ;;(setq *plan-env* (instance arm_planning_environment :init :robot (instance pr2-robot :init)))
  (setq *plan-env* (instance pr2torso_arm_planning_environment :init :robot (instance pr2-robot :init)))
  (send *ri* :arm-planning-environment *plan-env*)
  (setq (*plan-env* . robot) (send *ri* :robot)) ;; ???
  )

(defun demo-ik ()
  (setq cds (send (send *pr2* :rarm :end-coords) :copy-worldcoords))

  (send *pr2* :reset-pose)

  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *irtviewer* :draw-objects)
  (send *ri* :wait-interpolation)

  (send *ri* :collision-aware-ik cds :move-arm :rarm)

  (send *ri* :anglev-vector
        (send (send *ri* :arm-planning-environment) :robot :angle-vector))
  ;;(send *ri* :move-end-coords-plan cds :move-arm :rarm)

  (send *irtviewer* :draw-objects)
  )

(defun demo-ik-old ()
  (setq cds (send (send *pr2* :rarm :end-coords) :copy-worldcoords))

  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *irtviewer* :draw-objects)
  (send *ri* :wait-interpolation)

  (send *plan-env* :get-ik-for-pose cds :rarm
        :end-coords (list :rarm :end-coords))
  ;;(send *pr2* :angle-vector (send *plan-env* :robot :angle-vector))

  ;;(send *plan-env* :planning-move-arm :rarm)
  (setq traj (send *plan-env* :planning-make-trajectory :rarm))
  (when traj
    (send *ri* :joint-trajectory-to-angle-vector-list :rarm traj))

  (send *irtviewer* :draw-objects)
  )

(defun demo-ik-with-torso ()
  (setq cds (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send cds :translate #f(0 0 150) :world)

  (send *plan-env* :get-ik-for-pose cds :right-torso :end-coords (list :rarm :end-coords))
  (send *plan-env* :get-planning-scene)
  (send *plan-env* :planning-move-arm :right-torso)
  (send *irtviewer* :draw-objects)
  )

(defun demo-planning ()
  (warn ";; move to start position~%")
  (send *pr2* :rarm :angle-vector (float-vector -30.0 20.0 00.0 -30.0 00.0 -30.0 180.0))
  (send *plan-env* :planning-move-arm :rarm)

  (warn ";; add collision object~%")
  (let ((cyl (make-cylinder 120 2000)))
    (send cyl :translate (float-vector 600 -200 0))
    (send *collision-server* :add-object cyl :frame_id "/base_footprint"))
  (unix::sleep 2)

  (warn ";; start planning~%")
  (send *pr2* :rarm :angle-vector (float-vector 20.0 20.0 00.0 -30.0 00.0 -30.0 180.0))
  ;; (send *plan-env* :get-planning-scene)
  ;; (setq traj (send *plan-env* :motion-plan :rarm))
  ;; (send *ri* :joint-trajectory-to-angle-vector-list :rarm (send traj :trajectory))
  (send *plan-env* :planning-move-arm :rarm)
  )

(defun demo-planning-with-torso ()
  (warn ";; move to start position~%")
  (send *pr2* :rarm :angle-vector (float-vector -30.0 20.0 00.0 -30.0 00.0 -30.0 180.0))
  (send *plan-env* :planning-move-arm :right-troso)

  (warn ";; add collision object~%")
  (let ((cyl (make-cylinder 120 800)))
    (send cyl :translate (float-vector 600 -200 400))
    (send *collision-server* :add-object cyl :frame_id "/base_footprint"))
  (unix::sleep 2)

  (warn ";; start planning~%")
  (send *pr2* :rarm :angle-vector (float-vector 20.0 20.0 00.0 -30.0 00.0 -30.0 180.0))
  (send *plan-env* :planning-move-arm :right-troso)
  )
