#!/usr/bin/env roseus

(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(ros::roseus "publish_room_model_marker")
(ros::advertise "scene_marker_array" visualization_msgs::MarkerArray 1)

(if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))

(ros::rate 100)

(setq *scenename* (ros::get-param "~scene"))
(setq *frame* (ros::get-param "~frame_id"))
(setq *obj-id* (ros::get-param "~object_id"))

(unless *scenename* (setq *scenename* "room73b2"))
(unless *frame* (setq *frame* "/map"))
(unless *obj-id* (setq *obj-id* (string (gensym "PUBOBJ"))))


(load (format nil "models/~A-scene.l" *scenename*))
(setq *scene* (funcall (read-from-string *scenename*)))

(setq msg (instance visualization_msgs::MarkerArray :init))
(setq scene-marker-arrary nil)
(setq index-input 0)
(dolist (obj (send *scene* :objects))
  (setq header-msg (instance std_msgs::header :init
                             :stamp (ros::time-now) :frame_id (format nil "/scene_model_~d" index-input)))
  (unless (equal (send (class obj) :name) 'cascaded-coords)
    (print (list (send obj :name) (send obj :worldcoords)))

    (setq sp (make-sphere 1000 :pos (scale 1000 (float-vector 3 1 1000))))
    (setq sp-msg (sphere->marker-msg sp header-msg :alpha 0.5))
    (push sp-msg scene-marker-arrary)

    (send *tfb* :send-transform (send obj :worldcoords) *frame*  (format nil "/scene_model_~d" index-input))

    (push (object->marker-msg obj header-msg) scene-marker-arrary)
    (incf index-input)))
(send msg :markers scene-marker-arrary)
(ros::publish "scene_marker_array" msg)

(do-until-key
 (ros::sleep)
 (ros::spin-once)
 )

(unix::sleep 1)
(exit 0)
