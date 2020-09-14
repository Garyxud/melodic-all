;; eusmodel_to_urdf.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

;; assumed executed at PROJECT_SOURCE_DIR
(defvar *eusurdf-package-dir* (unix:getwd))
(require :conversion-utils (format nil "~A/euslisp/conversion-utils.l" *eusurdf-package-dir*))

(defun make-model-conf (name fpath &optional desc)
  (unless desc
    (setq desc (format nil "This is automatically generated eus model from ~A" name)))
  (with-open-file (f fpath :direction :output :if-exists :new-version)
    (format f "<?xml version='1.0'?>~%")
    (format f "<model>~%")
    (format f "  <name>~A</name>~%" name)
    (format f "  <version>0.1.0</version>~%")
    (format f "  <sdf>model.urdf</sdf>~%")
    (format f "  <description>~%")
    (format f "    ~A~%" desc)
    (format f "  </description>~%")
    (format f "</model>~%")))

(defun eusmodelfile2urdf (fname outdir &key (tmpdir "/tmp/") collada-to-urdf-exe-path)
  (unless (probe-file fname)
    (errorf "file ~A not exists" fname))
  (load fname)
  (let* ((model-name (string-join "-" (butlast (string-split (send (pathname fname) :name) #\-))))
         (model (funcall (read-from-string model-name))))
    (eusmodel2urdf model model-name outdir :tmpdir tmpdir :collada-to-urdf-exe-path collada-to-urdf-exe-path)))

(defun eusmodel2urdf (model model-name outdir &key (tmpdir "/tmp/") collada-to-urdf-exe-path)
  (let ((urdf-out-path (send (concatenate-pathnames outdir "model.urdf") :namestring))
        (mesh-out-dir (send (concatenate-pathnames outdir "meshes/") :namestring)))
    (cond ((null (send model :name)) (send model :name model-name))
          ((symbolp (send model :name)) (send model :name (string-downcase (send model :name)))))
    (collada::eus2collada model tmpdir)
    (make-dirs mesh-out-dir)
    (unless collada-to-urdf-exe-path
      (cond ((= (unix::system "rospack find collada_urdf_jsk_patch") 0)
	     (setq collada-to-urdf-exe-path "rosrun collada_urdf_jsk_patch collada_to_urdf"))
	    (t
	     (setq collada-to-urdf-exe-path "rosrun collada_urdf collada_to_urdf"))))

    (unix:system (format nil "~A ~A -G -A --mesh_output_dir ~A --mesh_prefix \"model://~A/meshes\" -O ~A"
                         collada-to-urdf-exe-path
                         (send (concatenate-pathnames tmpdir model-name ".dae") :namestring)
                         mesh-out-dir
                         model-name
                         urdf-out-path))
    (unix:system (format nil "rm -f ~A"
                         (send (concatenate-pathnames tmpdir model-name ".dae") :namestring)))
    (unix:system (format nil "sed -i -e \"s@continuous@revolute@g\" ~A" urdf-out-path))
    (unix:system (format nil "sed -i -e \"s@<robot name=\\\"inst_kinsystem\\\"@<robot name=\\\"~A\\\"@g\" ~A" model-name urdf-out-path))
    (unix:system (format nil "sed -i -e \"1,/  <link /s/  <link /  <gazebo>\\\n    <static>false<\\/static>\\\n  <\\/gazebo>\\\n  <link /\" ~A" urdf-out-path))
    (unix:system (format nil "sed -i -e \"s@      <inertia ixx=\\\"1e-09\\\" ixy=\\\"0\\\" ixz=\\\"0\\\" iyy=\\\"1e-09\\\" iyz=\\\"0\\\" izz=\\\"1e-09\\\"/>@      <inertia ixx=\\\"1e-03\\\" ixy=\\\"0\\\" ixz=\\\"0\\\" iyy=\\\"1e-03\\\" iyz=\\\"0\\\" izz=\\\"1e-03\\\"/>@g\" ~A" urdf-out-path))
    (make-model-conf model-name (format nil "~A/model.config" outdir))))

(when (string= "eusmodel_to_urdf"
               (send (pathname (cadr lisp::*eustop-argument*)) :name))
  ;; eval-when :execute
  (setq model-file-path (car (last (butlast (butlast lisp::*eustop-argument*)))))
  (setq output-directory (car (last (butlast lisp::*eustop-argument*))))
  (setq collada-to-urdf-exe-path (car (last lisp::*eustop-argument*)))
  (format t "converting eus object ~A -> ~A~%" model-file-path output-directory)
  (eusmodelfile2urdf model-file-path output-directory :collada-to-urdf-exe-path collada-to-urdf-exe-path)
  (exit))
(provide :eusmodel_to_urdf)
