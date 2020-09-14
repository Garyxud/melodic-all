
(ros::load-ros-manifest "pr2eus_moveit")

(load "package://pr2eus/pr2-interface.l");;
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l");;

(pr2)
(setq me (instance pr2-moveit-environment :init))
(ros::spin-once)

(setq pose-1 #f(168.25 0.0 0.0 0.0 -65.068 0.0 -60.1606 0.0 0.0 0.0 0.0 -65.068 0.0 -60.1606 0.0 0.0 0.0))
(setq pose-2 #f(168.25 0.0 0.0 0.0 -65.068 0.0 -60.1606 0.0 -30.9188 -18.6743 -32.5826 -121.542 101.376 -50.0787 -91.5661 0.0 0.0))
(send *pr2* :angle-vector pose-1)

;; move to pose-1
(send me :motion-plan :rarm)

(setq pose-constraint-1
      (make-pose-constraints "r_wrist_roll_link"
                             (send (car (last (send *pr2* :rarm :links))) :copy-worldcoords)
                             :frame_id "base_footprint" :use-position nil
                             :tolerance_rx 0.4 :tolerance_ry 0.4 :tolerance_rz 0.4))

(setq pose-constraint-2
      (make-pose-constraints "r_wrist_roll_link"
                             (send (car (last (send *pr2* :rarm :links))) :copy-worldcoords)
                             :frame_id "base_footprint" :use-position t
                             :tolerance_rx 0.4 :tolerance_ry 0.4 :tolerance_rz 0.4))

(setq pose-constraint-3
      (make-pose-constraints "r_wrist_roll_link"
                             (send (car (last (send *pr2* :rarm :links))) :copy-worldcoords)
                             :frame_id "world" :use-position t
                             :tolerance_rx 0.4 :tolerance_ry 0.4 :tolerance_rz 0.4))

(setq vj-constraint (make-virtual-joint-constraints (make-coords :pos (float-vector 400 0 0))))

(send me :motion-plan :rarm :path_constraints pose-constraint-1)
(send me :motion-plan :rarm :path_constraints pose-constraint-2)
(send me :motion-plan :rarm :path_constraints pose-constraint-3)

(send me :motion-plan-raw "base" :goal_constraints vj-constraint)

;;(send ms :motion-plan-raw "rarm-base"
;;      (merge-constraint pose-constraint-3 vj-constraint))
