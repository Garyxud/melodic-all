(defparameter *eusurdf-package-dir* (ros::resolve-ros-path "package://eusurdf"))
(require :eusmodel_to_urdf  "package://eusurdf/euslisp/eusmodel_to_urdf.l")
(require :eusscene_to_world "package://eusurdf/euslisp/eusscene_to_world.l")

(defun irteus2urdf-for-gazebo
  (model
   &key
   name
   eusurdf-package-path
   collada-to-urdf-exe-path
   (tmp-dir "/tmp/")
   )
  "Convert irteus model to urdf model. At first, irteus model is converted to collada model, and then collada model is converted to urdf model.
- model : irteus model instance
- name : name of urdf model. nil for using the name of irteus model.
- eusurdf-package-path : absolute path of eusurdf. nil for using rospack (this argument is necessary for using this function in catkin build.)
- collada-to-urdf-exe-path : absolute path of collada_to_urdf execution file. nil for using rospack (this argument is necessary for using this function in catkin build.)
- tmp-dir : temporal directory to put collada model.
- Typical usage:
    (progn (load \"models/laundry-machine-object.l\") (irteus2urdf-for-gazebo (laundry-machine) :name \"laundry\"))"
  (unless eusurdf-package-path
    (setq eusurdf-package-path (ros::resolve-ros-path "package://eusurdf")))
  (cond
   ((not (subclassp (class model) cascaded-link))
    (format t "invalid class, not childclass of cascaded-link~%")
    (return-from irteus2urdf-for-gazebo nil))
   ((null (send model :links))
    (format t "please setq links~%")
    (return-from irteus2urdf-for-gazebo nil))
   (t t))
  (unless name
    (cond ((or (not (stringp (send model :name)))
               (zerop (length (send model :name))))
           (setq name (remove #\: (format nil "~A" (send model :name)))))
          (t
           (setq name (send model :name))))
    )
  (let ((tmp-obj (instance (class model) :init))
        (urdf-output-directory
          (format nil "~A/models/~A/" eusurdf-package-path name)))
    (send tmp-obj :name name)
    (eusmodel2urdf tmp-obj name urdf-output-directory :tmpdir tmp-dir :collada-to-urdf-exe-path collada-to-urdf-exe-path))
  t)
(warn "(progn (load \"models/laundry-machine-object.l\") (irteus2urdf-for-gazebo (laundry-machine) :name \"laundry\"))~%")


(defun irteus2urdf-room-for-gazebo
  (room
   world-file-path
   &key
   make-static make-fixed
   eusurdf-package-path
   collada-to-urdf-exe-path
   (tmp-dir "/tmp/")
   )
  "Convert each model in irteus scene to urdf model. Generate world file for gazebo.
- room : irteus scene instance
- make-static : whether to generate urdf model with static property. (this is useful for gazebo.)
- make-fixed : whether to generate urdf model with fixed property. (this is useful for gazebo.)
- world-file-path : path of generated world file.
- eusurdf-package-path : absolute path of eusurdf. nil for using rospack (this argument is necessary for using this function in catkin build.)
- collada-to-urdf-exe-path : absolute path of collada_to_urdf execution file. nil for using rospack (this argument is necessary for using this function in catkin build.)
- tmp-dir : temporal directory to put collada model.
- Typical usage:
    (progn (load \"models/room73b2-scene.l\") (irteus2urdf-room-for-gazebo (room73b2) \"worlds/room73b2.world\"))"
  (unless world-file-path
    (error "world-file-path must not be empty"))
  (unless eusurdf-package-path
    (setq eusurdf-package-path (ros::resolve-ros-path "package://eusurdf")))
  (dolist (obj (send room :objects))
    (let ((obj-cnt 0) obj-name-suffix)

      ;; deal with the objects with no name. (objects with no name cause an error in converting)
      (cond ((null (send obj :name)) (send obj :name "no-name"))
            ((symbolp (send obj :name)) (send obj :name (string-downcase (send obj :name))))
            (t (send obj :name (string-downcase (send obj :name)))))

      ;; set obj-name
      (while t
       (setq obj-name-suffix (format nil "-~A" obj-cnt))
       (unless (eq 0
                   (unix:system (format nil "grep \"~A~A\" ~A > /dev/null"
                                        (send obj :name) obj-name-suffix
                                        world-file-path)))
         (return))
       (incf obj-cnt))

      ;; convert the model
      (when (irteus2urdf-for-gazebo obj :name (send obj :name) :eusurdf-package-path eusurdf-package-path :collada-to-urdf-exe-path collada-to-urdf-exe-path :tmp-dir tmp-dir)
        (add-model-include world-file-path obj :name-suffix obj-name-suffix :uri-suffix "_static")
        ;; ;; make the static model
        (when make-static
          (unix:system (format nil "~a/scripts/make_static_model.py ~a ~a"
                               eusurdf-package-path (send obj :name) eusurdf-package-path)))
        ;; ;; make the fixed model
        (when make-fixed
          (unix:system (format nil "~a/scripts/make_fixed_model.py ~a ~a"
                               eusurdf-package-path (send obj :name) eusurdf-package-path)))))))
(warn "(progn (load \"models/room73b2-scene.l\") (irteus2urdf-room-for-gazebo (room73b2) \"worlds/room73b2.world\"))~%")


(defun generate-room-models
  (name
   &key
   eusurdf-package-path
   collada-to-urdf-exe-path
   )
  "Generate urdf model and world file from the name of irteus scene.
- name : name of irteus scene (ex. room73b2)
- eusurdf-package-path : absolute path of eusurdf. nil for using rospack (this argument is necessary for using this function in catkin build.)
- collada-to-urdf-exe-path : absolute path of collada_to_urdf execution file. nil for using rospack (this argument is necessary for using this function in catkin build.)
- Typical usage:
    (generate-room-models \"room73b2\")"
  ;; check the function making room exits.
  (load (format nil "models/~a-scene.l" name))
  (unless (functionp (read-from-string name))
    (warn "[ERROR] room (~a) is not found.~%" name)
    (return-from generate-room-models))
  (unless eusurdf-package-path
    (setq eusurdf-package-path (ros::resolve-ros-path "package://eusurdf")))
  ;; check the model is derived from the room class
  (let ((room (funcall (read-from-string name)))
        (world-file-path (format nil "~a/worlds/~a.world" eusurdf-package-path name)))
    (unless (derivedp room scene-model)
      (warn "[ERROR] room (~a) is not the room model.~%" name)
      (return-from generate-room-models))
    ;; generate the world file
    (unless (probe-file (send (pathname world-file-path) :directory-string))
      (make-dirs (send (pathname world-file-path) :directory-string)))
    (write-empty-world-file world-file-path)
    ;; convert models
    (irteus2urdf-room-for-gazebo room world-file-path :make-static t :make-fixed t :eusurdf-package-path eusurdf-package-path :collada-to-urdf-exe-path collada-to-urdf-exe-path)))
(warn "(generate-room-models \"room73b2\")~%")
