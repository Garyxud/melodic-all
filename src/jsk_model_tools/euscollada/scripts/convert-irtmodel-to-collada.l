#!/usr/bin/env roseus

(ros::load-ros-manifest "euscollada")

(in-package "COLLADA")

(export '(convert-irtmodel-to-collada-ros))

;; use convert-irtmodel-to-collada function with ros::resolve-ros-path
(defun convert-irtmodel-to-collada-ros
  (model-file &optional (output-uri "package://euscollada") (model-name))
  "convert irtmodel to collada model. (convert-irtmodel-to-collada-ros irtmodel-file-path &optional (output-uri \"package://euscollada\") (model-name))"
  (collada::convert-irtmodel-to-collada model-file (ros::resolve-ros-path output-uri) model-name))

(in-package "USER")
(import '(collada::convert-irtmodel-to-collada-ros))
