(require :hrp2jsk-interface
         "package://jsk_hrpsys_ros_bridge/euslisp/hrp2jsk-interface.l")
(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(load "package://pr2eus_moveit/tutorials/collision-object-sample.l")

(defclass hrp2jsk-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod hrp2jsk-moveit-environment
  (:init
   (&key ((:robot rb) (hrp2jsk)))
   (send-super :init :robot rb
               :frame-id "BODY"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/odom")
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "rarm")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (remove (send robot :rarm_joint7) ;; gripper
                                         (send robot :rarm :joint-list)))
               )
         (list :larm
               (cons :group-name "larm")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (remove (send robot :larm_joint7) ;; gripper
                                         (send robot :larm :joint-list)))
               )
         (list :rarm-torso
               (cons :group-name "rarm_torso")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (remove (send robot :rarm_joint7)
                                                 (send robot :rarm :joint-list))))
               )
         (list :larm-torso
               (cons :group-name "larm_torso")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (remove (send robot :larm_joint7)
                                                 (send robot :larm :joint-list)))))
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
          (list "LARM_JOINT0" "LARM_JOINT1" "LARM_JOINT2" "LARM_JOINT3" "LARM_JOINT4" "LARM_JOINT5" "LARM_JOINT6"))
    (unix::usleep (* 100 1000))
    ))

(defun open-gripper (&optional (arm :larm))
  (send *hrp2jsk* arm :thumb-r :joint-angle -50))
(defun close-gripper (&optional (arm :larm))
  (send *hrp2jsk* arm :thumb-r :joint-angle -20))

#|
(require :hrp2jsk-interface
         "package://jsk_hrpsys_ros_bridge/euslisp/hrp2jsk-interface.l")
(hrp2jsk-init)
(setq *robot* *hrp2jsk*)
(send *ri* :angle-vector (send *robot* :angle-vector) 6000)

(send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 12 0 0) (float-vector 12 0 0)))
;;(send *ri* :set-st-param :k-tpcc-p #f(2.5 2.5) :k-tpcc-x #f(5 5) :k-brot-p #f(5 5) :k-brot-tc #f(0.1 0.1))
(send *ri* :set-gait-generator-param :default-double-support-ratio 0.2 :default-step-time 1.0 :default-step-height 0.08 :stride-parameter #f(0.15 0.04 30))

(progn (send *ri* :start-st)
       (send *ri* :go-pos 4 0 0)
       (send *ri* :stop-st))
|#

#|
;; pr2eus_moveit
(hrp2jsk-init)
(setq *robot* *hrp2jsk*)
(send *robot* :angle-vector (send *ri* :state :reference-vector))

(setq *me* (instance hrp2jsk-moveit-environment :init))
(send *me* :robot :angle-vector (send *ri* :state :reference-vector))

(setq cds (send *me* :robot :larm :end-coords :copy-worldcoords))
(send cds :translate #f(0 0 100) :world)
(send *me* :get-ik-for-pose cds :larm :end-coords (list :larm :end-coords))

(setq ret (send *me* :motion-plan :larm))
(send *me* :execute-trajectory (send ret :trajectory))
|#

#|
moveit_demo
(hrp2jsk-init)
(setq *robot* *sr*)
(init-pose)
(init-collision-demo)
(pub-objects)
(objects (append (list *robot*) (objects)))
(send *robot* :angle-vector (send *ri* :state :reference-vector))

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

(open-gripper)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)

(send *robot* :angle-vector (send *ri* :state :reference-vector))
(send *robot* :larm :inverse-kinematics
      (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 0 100) :world))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
|#