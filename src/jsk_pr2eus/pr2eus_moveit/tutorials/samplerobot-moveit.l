(require :samplerobot-interface
         "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(load "package://pr2eus_moveit/tutorials/collision-object-sample.l")

(defclass samplerobot-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod samplerobot-moveit-environment
  (:init
   (&key ((:robot rb) (samplerobot)))
   (send-super :init :robot rb
               :frame-id "WAIST_LINK0"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/odom")
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "rarm")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (remove (send robot :rarm_wrist_r)
                                         (send robot :rarm :joint-list)))
               )
         (list :larm
               (cons :group-name "larm")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (remove (send robot :larm_wrist_r)
                                         (send robot :larm :joint-list)))
               )
         (list :rarm-torso
               (cons :group-name "rarm_torso")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (remove (send robot :rarm_wrist_r)
                                                 (send robot :rarm :joint-list))))
               )
         (list :larm-torso
               (cons :group-name "larm_torso")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (remove (send robot :larm_wrist_r)
                                                 (send robot :larm :joint-list))))
               )
         )
   )
  )

(defun sync-larm (&optional (tm 500))
  (let ((av (send *ri* :state :reference-vector)))
    (send *ri* :robot :angle-vector av)
    (send *ri* :angle-vector av tm)
    (send *ri* :wait-interpolation)
    (send *ri* :remove-joint-group "larm")
    (unix::usleep (* 100 1000))
    (send *ri* :add-joint-group "larm"
          (list "LARM_SHOULDER_P" "LARM_SHOULDER_R" "LARM_SHOULDER_Y" "LARM_ELBOW" "LARM_WRIST_Y" "LARM_WRIST_P"))
    (unix::usleep (* 100 1000))
    ))

(defun open-gripper (&optional (arm :larm))
  (send *sr* arm :wrist-r :joint-angle -40))
(defun close-gripper (&optional (arm :larm))
  (send *sr* arm :wrist-r :joint-angle -12))

(defun init-pose ()
  (send *sr* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -10.0 0.0 -60.0 9.0 -6.5 25.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 15.0 0.0 -60.0 -9.0 -6.5 -40.0 0.0 0.0 0.0))
  (open-gripper :larm)
  (send *sr* :fix-leg-to-coords (make-coords :pos #f(-450 0 0)))
  (send *ri* :angle-vector (send *sr* :angle-vector) 6000)
  )

#|
;; pr2eus_moveit
(samplerobot-init)

(send *robot* :angle-vector (send *ri* :state :reference-vector))

(setq *me* (instance samplerobot-moveit-environment :init))
(send *me* :robot :angle-vector (send *ri* :state :reference-vector))

(setq cds (send *me* :robot :larm :end-coords :copy-worldcoords))
(send cds :translate #f(0 0 100) :world)
(send *me* :get-ik-for-pose cds :larm :end-coords (list :larm :end-coords))

(setq ret (send *me* :motion-plan :larm))
(send *me* :execute-trajectory (send ret :trajectory))
|#

#|
moveit_demo
(samplerobot-init)
(setq *robot* *sr*)
(init-pose)
(init-collision-demo)
(pub-objects)
(objects (append (list *robot*) (objects)))
(send *robot* :angle-vector (send *ri* :state :reference-vector))
(send *irtviewer* :draw-objects)
;;
(setq larm-target #s(coordinates plist nil rot #2f((0.968121 0.247905 0.035856) (-0.247379 0.968742 -0.018505) (-0.039323 0.009045 0.999186)) pos #f(-81.4698 172.586 808.425)))
(setq *me* (instance samplerobot-moveit-environment :init))
(send *me* :sync-robot-model *robot*)
(send *me* :get-ik-for-pose larm-target :larm :end-coords (list :larm :end-coords))

(setq ret (send *me* :motion-plan :larm))
(send *me* :execute-trajectory (send ret :trajectory))
;;
;; OR
;;
execute plan in rviz
;;
;;

(send *robot* :angle-vector (send *ri* :state :reference-vector))
(send *irtviewer* :draw-objects)
(sync-larm)

(close-gripper)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)

(send *robot* :angle-vector (send *ri* :state :reference-vector))
(send *robot* :larm :inverse-kinematics
      (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 0 100) :world))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
|#
