#!/usr/bin/env roseus


;;  convert eus object model to moveit scene
;;;; example
;;;; (load "models/foldable-desk-object.l")
;;;; (generate-scene-file-from-eusobject (foldable-desk))
(defun generate-scene-file-from-eusobject
  (obj &optional (filename (format nil "./~A.scene" (send obj :name))))

  (with-open-file
   (f filename :direction :output)
   (format f "scene_~A~%" (send obj :name))
   (dolist (obj (list obj))
     (_convert-obj-to-scene-format obj f)
     )
   (format f ".~%")
   ))

;;  convert eus scene model to moveit scene
;;;; example
;;;; (load "models/room73b2-scene.l")
;;;; (generate-scene-file-from-eusscene (room73b2))
(defun generate-scene-file-from-eusscene
  (scene &optional (filename (format nil "./~A.scene" (send scene :name))))

  (with-open-file
   (f filename :direction :output)
   (format f "scene_~A~%" (send scene :name))
   (dolist (obj (send scene :objects))
     (_convert-obj-to-scene-format obj f)
     )
   (format f ".~%")
   ))


(defun _convert-obj-to-scene-format
  (obj file)

  ;; skip cascoords
  (when (or (find-method obj :bodies) (find-method obj :faces))
    (let (bods)
      (if (find-method obj :bodies)
          (setq bods (send obj :bodies))
        (setq bods (list obj)))
      (format file "* model_~A~%" (send obj :name))
      (format file "~A~%" (length bods))
      (dolist (b bods)
        (setq pos (scale 0.001 (send b :worldpos)))
        (setq qut (matrix2quaternion (send b :worldrot)))
        (cond ((and (find-method b :csg) (assoc :cube (send b :csg)))
               (setq csg (scale 0.001 (coerce (cdr (assoc :cube (send b :csg))) float-vector)))
               (format file "box~%")
               (format file "~A ~A ~A~%" (elt csg 0) (elt csg 1) (elt csg 2))
               )
              ((and (find-method b :csg) (assoc :cylinder (send b :csg)))
               (setq csg (scale 0.001 (coerce (cdr (assoc :cylinder (send b :csg))) float-vector)))
               (format file "cylinder~%")
               (format file "~A ~A~%" (elt csg 0) (elt csg 1))
               (setq pos (v+ pos (float-vector 0 0 (/ (elt csg 1) 2))))
               )
              (t
               (let (vers vs v1 v2 v3 r)
                 (setq r (send b :worldcoords))
                 (format file "mesh~%")
                 (dolist (file (send b :faces))
                   (dolist (poly (geo::face-to-triangle-aux file))
                     (setq vs (send poly :vertices)
                           v1 (scale 0.001 (send r :inverse-transform-vector (first vs)))
                           v2 (scale 0.001 (send r :inverse-transform-vector (second vs)))
                           v3 (scale 0.001 (send r :inverse-transform-vector (third vs))))
                     (push (list v1 v2 v3) vers)))
                 (format file "~A ~A~%" (* (length vers) 3) (length vers))
                 (dolist (v (nreverse vers))
                   (let ((v1 (elt v 0)) (v2 (elt v 1)) (v3 (elt v 2)))
                     (format file "~A ~A ~A~%" (elt v1 0) (elt v1 1) (elt v1 2))
                     (format file "~A ~A ~A~%" (elt v2 0) (elt v2 1) (elt v2 2))
                     (format file "~A ~A ~A~%" (elt v3 0) (elt v3 1) (elt v3 2))))
                 (dotimes (i (length vers))
                   (format file "~A ~A ~A~%" (+ (* i 3) 0) (+ (* i 3) 1) (+ (* i 3) 2)))
                 ))
              ) ;; cond
        (format file "~A ~A ~A~%" (elt pos 0) (elt pos 1) (elt pos 2))
        (format file "~A ~A ~A ~A~%" (elt qut 1) (elt qut 2) (elt qut 3) (elt qut 0))
        (setq col (get b :face-color))
        (unless col (setq col (float-vector 0 0 1 1)))
        (format file "~A ~A ~A ~A~%" (elt col 0) (elt col 1) (elt col 2)
                (if (> (length col) 3) (elt col 3) 1.0))
        ))))
