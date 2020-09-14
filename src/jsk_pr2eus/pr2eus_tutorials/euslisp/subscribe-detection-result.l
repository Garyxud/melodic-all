#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;;
(defvar *typename* "perception_result")

;; パッケージ読み込み
(ros::roseus-add-msgs "posedetection_msgs")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "sub_detection_result")

;;矢印読み込み
(load "models/arrow-object.l")
(setq *ar* (arrow))

;; transform listener
(setq *tfl* (instance ros::transform-listener :init))

;;
;; object detection result subscribe
;;
(setq *cds* nil)
(defun objectdetection-cb (type msg)
  (let (frame-id (obj (car (send msg :objects))))
    (describe obj)
    (when (and obj
               (string= type (send obj :type)))
      (let ((fid (send msg :header :frame_id)))
        (if (and fid
                 (/= (elt fid 0) #\/))
            (setq frame-id (concatenate string "/" fid))
          (setq frame-id fid)))

      (setq *cds*
            (send *tfl* :lookup-transform
                  "/base_footprint" frame-id (send msg :header :stamp)))
      (when *cds*
        (send *cds* :transform (ros::tf-pose->coords (send obj :pose))))
      )
    ))

(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection
                #'objectdetection-cb *typename*)

(defun proc-detection (&optional (max-count 100))
  (let ((counter 0))
    (setq *cds* nil)
    (ros::rate 10)
    (while t
      (ros::spin-once)
      (when *cds*
        (return-from proc-detection *cds*))
      (if (> counter max-count) (return-from proc-detection))
      (incf counter)
      (ros::sleep))
    ))

;;
;; point cloud subscribe
;;
(setq *points* nil)
(defun callback-points ( msg )
  (setq *points*
        (make-eus-pointcloud-from-ros-msg
         msg :remove-nan t)) ;; for replace -> :remove-nan :replace
  (if (and *points* (> (send *points* :size) 0))
      (setf (get *points* :header) (send msg :header))
    (setq *points* nil))
  *points*)

(ros::subscribe "/openni/depth_registered/points"
                sensor_msgs::PointCloud2
                #'callback-points)

(defun proc-points (&optional (max-count 20))
  (let ((counter 0))
    (setq *points* nil)
    (ros::rate 10)
    (while t
      (ros::spin-once)
      (when *points*
        (return-from proc-points *points*))
      (if (> counter max-count) (return-from detect-points))
      (incf counter)
      (ros::sleep))
    ))

(pr2)
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *ri* (instance pr2-interface :init))
;;(send *pr2* :reset-pose)
;;ボードの座標
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(objects (list *pr2* *ar*))
(send *irtviewer* :draw-objects)

#|
;;
;;
(let (cds)
  (setq cds (proc-detection))
  (when cds
    (send *ar* :reset-pose)
    (send *ar* :transform cds)
    (send *pr2* :head :look-at cds)
    (send *pr2* :larm :inverse-kinematics cds)
    (send *irtviewer* :draw-objects))
  )
|#
