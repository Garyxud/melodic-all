(ros::load-ros-manifest "pr2eus_moveit")
(load "package://jsk_perception/euslisp/detection_interface.l")
(load "package://pr2eus/pr2-interface.l");;
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l");;

;;(setq *ri* (instance pr2-interface :init))
;;(pr2)
(pr2-init)
(send *ri* :set-moveit-environment (instance pr2-moveit-environment :init))

(defun go-to-default-pose ()
  (send *pr2* :angle-vector
        #f(50.0 60.0 74.0 90.0 -120.0 20.0 -50.0 180.0 -60.0 74.0 -90.0 -120.0 -20.0 -50.0 180.0 0.0 48.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000))
;;#f(50.0 60.0 74.0 90.0 -120.0 20.0 -50.0 180.0 -60.0 74.0 -90.0 -120.0 -20.0 -50.0 180.0 0.0 48.0)

(defun tabletop-detection (&optional (timeout 30))
  (send *pr2* :angle-vector (send *ri* :state :reference-vector))
  (send *pr2* :head :neck-p :joint-angle 35)
  (send *pr2* :head :neck-y :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
  (send *ri* :wait-interpolation)
  (unix::usleep (* 800 1000))
  (check-detection :type "tabletop0"
                   :detection-topic "/tabletop/ObjectDetection"
                   :timeout timeout)
  )

(defun grasp-object-on-table (target-coords &key (use-torso t) (use-arm :rarm))
  (let ((cds-org (send target-coords :copy-worldcoords))
        (cds-tgt (send target-coords :copy-worldcoords))
        ret)

    ;;(setq cds-org (send target-coords :copy-worldcoords))
    ;;(setq cds-tgt (send target-coords :copy-worldcoords))
    (send cds-tgt :translate (float-vector -120 0 40))
    (send *ri* :stop-grasp use-arm) ;; open gripper

    ;; moveit
    (setq ret
          (send *ri* :move-end-coords-plan cds-tgt
                :move-arm use-arm :use-torso use-torso))
    ;; error check
    (unless ret
      (warn ";;; Move It failed ... ;;;~%")
      (return-from grasp-object-on-table))
    (send *ri* :wait-interpolation)

    ;;
    (send cds-org :translate (float-vector 20 0 10))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* use-arm :inverse-kinematics cds-org)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
    (send *ri* :wait-interpolation)
    ;; grasp
    (send *ri* :start-grasp use-arm)
    (send *ri* :wait-interpolation)

    ;; pickup
    (send cds-org :translate (float-vector 0 0 70))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* use-arm :inverse-kinematics cds-org)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
    (send *ri* :wait-interpolation)

    ;; go-to-default-pose
    (send *ri* :angle-vector-motion-plan
          #f(50.0 60.0 74.0 90.0 -120.0 20.0 -50.0 180.0 -60.0 74.0 -90.0 -120.0 -20.0 -50.0 180.0 0.0 48.0)
          :move-arm use-arm :use-torso use-torso)
    ))

(warn "
;;(go-to-default-pose)
;;(setq cds (tabletop-detection))
;;(grasp-object-on-table cds)
")
