;; eusscene_to_xacro.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

;; assumed executed at PROJECT_SOURCE_DIR
(defparameter *eusurdf-package-path* (unix:getwd))
(require :conversion-utils (format nil "~A/euslisp/conversion-utils.l" *eusurdf-package-path*))

(defun write-xacro-file-header (s name root-link-name)
  (format s "<?xml version=\"1.0\" ?>~%~%")
  (format s "<robot name=\"~A\" xmlns:xacro=\"http://ros.org/wiki/xacro\">~%~%" name)
  (format s "  <macro name=\"sphere_inertia\" params=\"radius mass\">~%")
  (format s "    <inertial>~%")
  (format s "      <origin xyz=\"0.0 0.0 0.0\" rpy=\"0 0 0\"/>~%")
  (format s "      <mass value=\"1\" />~%")
  (format s "      <inertia ixx=\"1\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1\" iyz=\"0.0\" izz=\"1\"/>~%")
  (format s "    </inertial>~%")
  (format s "  </macro>~%")
  (format s "  <link name=\"~A\">~%" root-link-name)
  (format s "    <sphere_inertia radius=\"0\" mass=\"0\"/>~%")
  (format s "  </link>~%")
  (format s "  <gazebo>~%")
  (format s "    <static>false</static>~%")
  (format s "  </gazebo>~%")
)

(defun write-xacro-file-footer (s)
  (format s "</robot>~%"))

(defun write-model-include-xml-stirng (s model model-name-suffix xacro-path parent-link &optional model-name (indent 2))
  (unless model-name (setq model-name (send model :name)))
  (let ((pos-string (string-join " " (coerce (scale 0.001 (send model :worldpos)) cons)))
        (rpy-string (string-join " " (reverse (coerce (car (rpy-angle (send model :worldrot))) cons))))
        (indent-string (make-indent-string indent)))
    (format s "~A<xacro:include filename=\"~A\"/>~%" indent-string xacro-path)
    (format s "~A<~A~A name=\"~A\" parent=\"~A\">~%" indent-string
            (send model :name) model-name-suffix model-name parent-link)
    (format s "~A  <origin xyz=\"~A\" rpy=\"~A\"/>~%" indent-string pos-string rpy-string)
    (format s "~A</~A~A>~%" indent-string (send model :name) model-name-suffix)))

(defun eusscene2xacro (scene-file-name xacro-file-name)
  (unless (probe-file scene-file-name)
    (errorf "file ~A not exists" scene-file-name))
  (load scene-file-name)
  (let* ((scene-name (string-join "-"
                                  (butlast
                                   (string-split (send (pathname scene-file-name) :name) #\-))))
         (scene (funcall (read-from-string scene-name)))
         (root-link-name (format nil "~A_root_link" scene-name))
         (model-count (make-hash-table)))
    (make-dirs (send (pathname xacro-file-name) :directory-string))
    (with-open-file (f xacro-file-name :direction :output :if-exists :new-version)
      (write-xacro-file-header f scene-name root-link-name)
      (dolist (model (remove-if-not #'(lambda (x)
                                        (and (subclassp (class x) cascaded-link)
                                             (not (null (send x :links)))))
                                    (send scene :objects)))
        ;; stringify model name
        (cond ((null (send model :name)) (send model :name "no-name"))
              ((symbolp (send model :name)) (send model :name (string-downcase (send model :name)))))

        ;; include model xacro file
        (let ((model-name-key (read-from-string (send model :name)))
              (model-name-suffix "")
              model-name xacro-path)
          (if (gethash model-name-key model-count)
              (incf (gethash model-name-key model-count))
              (setf (gethash model-name-key model-count) 0))
          (setq model-name (string-join "_"
                                        (list
                                         (send model :name)
                                         (gethash model-name-key model-count))))
          (case (use-textured-model-p (send model :name))
            (:fixed
             (setq model-name-suffix "_fixed")
             (setq xacro-path
                   (format nil "$(find eusurdf)/textured_models/~A_fixed/model.urdf.xacro" (send model :name))))
            (:static
             (setq model-name-suffix "_static")
             (setq xacro-path
                   (format nil "$(find eusurdf)/textured_models/~A_static/model.urdf.xacro" (send model :name))))
            (nil
             (setq xacro-path
                   (format nil "$(find eusurdf)/models/~A/model.urdf.xacro" (send model :name))))
            (t
             (setq xacro-path
                   (format nil "$(find eusurdf)/textured_models/~A/model.urdf.xacro" (send model :name)))))
          (write-model-include-xml-stirng f model model-name-suffix xacro-path root-link-name model-name)))
      (write-xacro-file-footer f))
    ))

(setq scene-file-path (car (last (butlast lisp::*eustop-argument*)))))
(setq xacro-file-path (car (last lisp::*eustop-argument*))))
(format t "converting eus scene ~A -> ~A~%" scene-file-path xacro-file-path)
(eusscene2xacro scene-file-path xacro-file-path)
(exit)
