#!/usr/bin/env roseus

(ros::roseus-add-msgs "posedetection_msgs")

(load "package://pr2eus_tutorials/euslisp/pr2main.l")
;; (load "package://pr2eus_tutorials/arm-navigation-sample.l")

(defun detect-callback ( msg )
  (setq *msg* msg))

(setq *msg* nil)
(ros::roseus "tabletop_object_sample")
;; /ObjectDetection にテーブル上の物体の位置が出ている
(ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection
                #'detect-callback)

;; initialize PR2 ROS-euslisp interface
(pr2-init t)
;; goto initial pose
(goto-init-pose)
(send *irtviewer* :draw-objects)

;;
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

;; /ObjectDetection に出ている位置を受け取る
(defun detect-one (&optional (timeout 500))
  "subscribe just one detection result and return list of worldcoords of object's centroid"
  (let (lst
        (cntr 0)
        trans-base->table)
    (setq *msg* nil)

    (ros::rate 10)
    (while (null *msg*)
      (ros::spin-once)
      (ros::sleep)
      (if (> (incf cntr) timeout)
          (return-from detect-one nil)))

    (setq trans-base->table
          (send *tfl* :lookup-transform
                "/base_footprint"
                (send *msg* :header :frame_id)
                (send *msg* :header :stamp)))

    (dolist (obj (send *msg* :objects))
      (let ((trs
             (send (send trans-base->table :copy-worldcoords)
                   :transform (ros::tf-pose->coords (send obj :pose)))))
        (setf (get trs :type) (send obj :type))
        (push trs lst)
        ))
    lst))

(unix::sleep 5) ;; waiting detection

(defun pickup-pick (&key (arm :rarm))
  (let ((av-st (send *pr2* :angle-vector))
        av-ed av-mid)
  (setq *res* (detect-one))

  ;; sort nearest first// 手に近い物体から順番に並べる
  (sort *res*
        #'(lambda (x y)
            (<= (norm (send (send (send *pr2* arm :end-coords :worldcoords) :transformation x) :pos))
                (norm (send (send (send *pr2* arm :end-coords :worldcoords) :transformation y) :pos)))))
  (cond
   ((send *pr2* arm :inverse-kinematics (car *res*) :rotation-axis :z)
    ;;(setq av-ed (send *pr2* :angle-vector))
    ;; 物体をつかむ位置へ手先を持っていくIK(inverse kinematics/逆運動学)
    (let ((cds (send *pr2* arm :end-coords :copy-worldcoords)))
      (send cds :translate (float-vector 25 0 10 )) ;; 25mm offset of centroid
      (if (send *pr2* arm :inverse-kinematics cds :rotation-axis :z)
          (setq av-ed (send *pr2* :angle-vector))))

    ;; 物体をつかむ位置へのアプローチのIK(inverse kinematics/逆運動学)
    (let ((cds (send *pr2* arm :end-coords :copy-worldcoords)))
      (send cds :translate (float-vector -75 0 0)) ;; linear reach to object
      (if (send *pr2* arm :inverse-kinematics cds)
          (setq av-mid (send *pr2* :angle-vector))))
    (send *irtviewer* :draw-objects)
    (when av-mid
      ;; 手を開く
      (send *ri* :stop-grasp arm)
      (send *ri* :wait-interpolation) ;; wait for finising action
      ;; 腕を動かす
      (send *ri* :angle-vector-sequence
            (list av-mid av-ed) (list 3800 1200))
      (send *ri* :wait-interpolation) ;; wait for finising action

      ;; 手を閉じる
      ;;(> (send *ri* :start-grasp arm) 8) ;; hand width is more than 8mm
      ;; gripper action not returned while have not reached goal position ???
      (send *ri* :move-gripper arm 0.03 :wait nil)
      (unix::usleep (* 2 1000 1000))
      )
    )
   (t nil))
  (send *irtviewer* :draw-objects)
  ))

(defun pickup-up (&key (arm :rarm))
  ;; 手を少し上げる
  (send *pr2* arm :move-end-pos (float-vector 0 0 55) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2500)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  ;; 元の姿勢に戻る
  (goto-init-pose)
  (send *irtviewer* :draw-objects)
  )

(warn ";;
;; (pickup-pick :arm :rarm)
;; (pickup-up :arm :rarm)
;; (pickup-pick :arm :larm)
;; (pickup-up :arm :larm)
;;
")
