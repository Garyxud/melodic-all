;; eusmodel-description.l
;;
;; this file provides an interface to euslisp model IForm.
;; in eusmodel_converter, all of the euslisp models (subclass of robot-frame),
;; will be converted into model-iform. model-iform is a simple associated-list.

;; model-iform spec:
;;  model-iform := (<link-specs> . <joint-specs>)
;;  <link-specs> := (<link-spec> ...)
;;  <joint-specs> := (<joint-spec> ...)
;;  <link-spec> := ((:name . <string>)
;;                  (:origin . <coordinates>)
;;                  (:mesh . <mesh-spec>))
;;  <mesh-spec> := (<triangle faceset>)
;;   (all of the vertices has relative coordinates from the origin of the link)
;;  <joint-spec> := ((:name . <string>)
;;                   (:type . <joint-type>)
;;                   (:limit . <limit-spec>)
;;                   (:axis . <float-vector>)
;;                   (:parent . <string>)
;;                   (:child . <string>))
;;  <joint-type> := :rotational-joint|linear-joint
;;  <limit-spec> := ((:lower . <float>)
;;                   (:upper . <float>))

(defun eusmodel-description (model)
  "convert a `model' to eusmodel-description"
  ;; FIXME: make robots reset
  (let ((links (send model :links))
        (joints (send model :joint-list)))
    ;; verification
    (verificate-unique-strings (mapcar #'string (send-all links :name)))
    (verificate-unique-strings (mapcar #'string (send-all joints :name)))
    (list (eusmodel-link-specs links)
          (eusmodel-joint-specs joints)
          (eusmodel-endcoords-specs model))))

(defun verificate-unique-strings (names)
  (if (not (= (length names)
              (length (remove-duplicates names :test #'string=))))
      (error "NOT UNIQUE NAMES ~A" names)
    t))

(defun eusmodel-link-specs (links)
  "convert `links' to <link-specs>"
  (mapcar #'eusmodel-link-spec links))

(defun eusmodel-joint-specs (joints)
  "convert `joints' to <joint-specs>"
  (mapcar #'eusmodel-joint-spec joints))

(defun eusmodel-link-spec (link)
  (list (cons :name (string (send link :name)))
        (cons :origin                   ;parent -> child transformation
              (let ((parent (send link :parent)))
                (send parent :transformation link)))
        (cons :mesh (eusmodel-mesh-spec link))))

(defun eusmodel-mesh-spec (link)
  ;; right?
  (let ((bs (send link :bodies)))
    (mapcar #'(lambda (b)
                (let ((trimesh
                       (instance faceset :init
                                 :faces
                                 (send (geo::body-to-triangles b nil)
                                       :faces))))
                  (setf (get trimesh :face-color) (get b :face-color))
                  ;; move trimesh to origin...
                  (send trimesh
                        :move-to (send (send b :copy-worldcoords)
                                       :inverse-transformation))
                  trimesh))
            bs)))

(defun eusmodel-joint-spec (_joint)
  (list (cons :name (string (send _joint :name)))
        (cons :type (cond
                     ((derivedp _joint rotational-joint)
                      :rotational-joint)
                     ((derivedp _joint linear-joint)
                      :linear-joint)))
        (cons :limit (eusmodel-limit-spec _joint))
        (cons :axis (case (_joint . axis)
                      (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                      (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                      (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                      (t (if (listp (_joint . axis))
                             ;; 6dof joint...
                             #f(0 0 0)
                             (_joint . axis)))))
        (cons :parent (string (send (send _joint :parent-link) :name)))
        (cons :child (string (send (send _joint :child-link) :name)))))

(defun eusmodel-limit-spec (_joint)
  (list (cons :upper (deg2rad (send _joint :max-angle)))
        (cons :lower (deg2rad (send _joint :min-angle)))))


(defun eusmodel-endcoords-specs (model)
  (labels ((get-links
            (link)
            (if (find-method model link)
                (let ((tmp (send model link)))
                  (if (and tmp (send model link :end-coords)
                           (send (send model link :end-coords) :parent))
                      (let ((link-list
                             (send model :link-list
                                   (send (send model link :end-coords) :parent)
                                   (car tmp))))
                        (cons (send (car tmp) :parent-link)
                              link-list)))))))
    (let ((head (get-links :head))
          (larm (get-links :larm))
          (rarm (get-links :rarm))
          (lleg (get-links :lleg))
          (rleg (get-links :rleg)))
      (list (cons "head"
                  (mapcar #'string (send-all head :name)))
            (cons "larm"
                  (mapcar #'string (send-all larm :name)))
            (cons "rarm"
                  (mapcar #'string (send-all rarm :name)))
            (cons "lleg"
                  (mapcar #'string (send-all lleg :name)))
            (cons "rleg"
                  (mapcar #'string (send-all rleg :name)))))))

;; accessor
(defun eusmodel-link-description (description)
  (car description))

(defun eusmodel-joint-description (description)
  (cadr description))

(defun eusmodel-endcoords-description (description)
  (caddr description))
