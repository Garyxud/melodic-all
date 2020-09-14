#!/usr/bin/env roseus

(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(setq *co* (instance collision-object-publisher :init))

(setq *obj* nil)
(setq *objname* (ros::get-param "~object"))
(setq *frame* (ros::get-param "~frame_id"))
(setq *trans* (ros::get-param "~translation"))
(setq *rot* (ros::get-param "~rotation"))
(setq *obj-id* (ros::get-param "~object_id"))

(unless *objname* (setq *objname* "foldable-desk"))
(unless *frame* (setq *frame* "/map"))
(unless *obj-id* (setq *obj-id* (string (gensym "PUBOBJ"))))

(if *trans*
    (setq *trans*
          (scale 1000.0 (eval (read-from-string (format nil "(float-vector ~A)" *trans*)))))
  (setq *trans* (float-vector 0 0 0)))
(if *rot*
    (setq *rot* (eval (read-from-string (format nil "(float-vector ~A)" *rot*))))
  (setq *rot* (float-vector 1 0 0 0)))

(setq *coords*
      (make-coords :pos *trans*
                   :rot (quaternion2matrix *rot*)))

(when *objname*
  (load (format nil "models/~A-object.l" *objname*))
  (setq *obj* (funcall (read-from-string *objname*))))

(unix::sleep 1)

(if *obj*
    (send *co* :add-object *obj* :frame-id *frame* :relative-pose *coords*)
  (ros::ros-warn "There is no object named ~A" *objname*))

(unix::sleep 1)
(exit 0)
