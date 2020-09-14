#!/usr/bin/env roseus

(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(setq *co* (instance collision-object-publisher :init))

(setq *scenename* (ros::get-param "~scene"))
(setq *frame* (ros::get-param "~frame_id"))
(setq *obj-id* (ros::get-param "~object_id"))

(unless *scenename* (setq *scenename* "room73b2"))
(unless *frame* (setq *frame* "/map"))
(unless *obj-id* (setq *obj-id* (string (gensym "PUBOBJ"))))


(load (format nil "models/~A-scene.l" *scenename*))
(setq *scene* (funcall (read-from-string *scenename*)))

(dolist (obj (send *scene* :objects))
  (unless (equal (send (class obj) :name) 'cascaded-coords)
    (print (list (send obj :name) (send obj :worldcoords)))
    ;;(unix::usleep (* 100 1000))
    (send *co* :add-object obj :frame-id *frame* :relative-pose (send obj :copy-worldcoords)))
  )

(unix::sleep 1)
(exit 0)
