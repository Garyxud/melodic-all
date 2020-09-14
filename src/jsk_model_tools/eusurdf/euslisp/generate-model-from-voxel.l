#!/usr/bin/env roseus

(load "package://eusurdf/euslisp/convert-eus-to-urdf.l")

(ros::roseus "generate_model_from_voxel")

(ros::roseus-add-srvs "jsk_pcl_ros")

(defclass simple-model
  :super cascaded-link
  :slots ())
(defmethod simple-model
  (:init
   (&key ((:bodies bodies_) *bodies*))
   (let (b)
     (send-super :init)
     (setq b (instance bodyset-link :init (make-cascoords) :bodies bodies_))
     (setq links (list b))
     (send self :assoc b)
     (send self :init-ending)
     self))
  )

(defun generate-eus-model
  (req &key (draw? nil))
  (let* ((res (send req :response))
         (marker (send req :voxel))
         (points (send marker :points))
         (marker-scale (send marker :scale))
         (marker-color (send marker :color))
         (name (send req :name))
         (filename (send req :filename))
         cube-list
         tmp-cube
         obj
         )
      ;; make cube-list ;; unit is converted from [m] to [mm]
      (dolist (p points)
        (setq tmp-cube (make-cube (* (send marker-scale :x) 1000) (* (send marker-scale :y) 1000) (* (send marker-scale :z) 1000)))
        (send tmp-cube :locate (ros::tf-point->pos p))
        (send tmp-cube :set-color (float-vector (send marker-color :r) (send marker-color :g) (send marker-color :b)))
        (push tmp-cube cube-list)
        )
      ;; generate object
      (setq *bodies* cube-list)
      (setq obj (instance simple-model :init))
      (irteus2urdf-for-gazebo obj :name name)
      ;; copy urdf model to the specified file
      (unix:system (format nil "cp ~a ~a"
                           (format nil (ros::resolve-ros-path "package://eusurdf/models/~a/model.urdf") name)
                           filename))
      ;; draw
      (when draw?
        (objects (list obj))
        (x::window-main-one))
      (send res :result t)
    res)
  )

(defun main
  ()
  (ros::advertise-service "/generate_voxel_eus_model" jsk_pcl_ros::VoxelModelGenerate #'generate-eus-model)
  (ros::spin)
  )

(main)
