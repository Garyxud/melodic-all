(unless (fboundp 'stl2eus) (load "read-stl.l"))

(defun massoc (alist &rest args)
  (if (null args)
      alist
    (apply #'massoc (assoc (car args) alist) (cdr args))))

(defun sxml-attribute (sxml) (assoc '@ sxml))

(defun read-list-from-xml-value (str)
  (let ((input (make-string-input-stream str)))
    (let ((tokens nil)
          (token nil))
      (while (setq token (read input nil nil))
        (push token tokens))
      (reverse tokens))))

(defun read-float-vector-from-xml-value (str)
  (coerce (read-list-from-xml-value str) float-vector))

(defun resolve-ros-path (fname)
  (let* ((package-name (cadr (pathname-directory fname)))
         (package-rest (namestring (make-pathname :directory (cddr (pathname-directory fname)) :name (pathname-name fname) :type (pathname-type fname)))))
    (format nil "~A/~A"
            (ros::rospack-find package-name)
            package-rest)))

(defun make-geometry-from-urdf (sxml &key
                                     (coords (make-coords))
                                     (name "unknown")
                                     (scale 1000.0))
  (let ((specifier (caar (remove-if #'(lambda (x) (eq (car x) '@)) (cdr sxml))))
        (contents (cdar (remove-if #'(lambda (x) (eq (car x) '@)) (cdr sxml)))))
    (when sxml
      (case specifier
        (mesh
         (let ((fname (cadr (massoc contents '@ 'filename))))
           (let ((path (resolve-ros-path fname)))
             (stl2eus path :coords coords :scale scale :name name))))
        (box
         (let ((val (mapcar #'(lambda (x) (* scale x))
                            (read-list-from-xml-value
                             (cadr (massoc contents '@ 'size))))))
           (make-cube (elt val 0) (elt val 1) (elt val 2) :coords coords :name name)))
        (cylinder
         (let ((radius (* scale (read-from-string
                                 (cadr (massoc contents '@ 'radius)))))
               (length (* scale (read-from-string
                                 (cadr (massoc contents '@ 'length))))))
           (make-cylinder radius length :coords coords :name name)))
        (sphere
         (warn "spehre does not supported~%")
         (make-cube 1 1 1))
        (t
         (error "~A is not supported" sxml))))))

(defun make-origin-coords (origin-sxml &key (scale 1000.0))
  (let ((origin-xyz (cadr (massoc origin-sxml '@ 'xyz)))
        (origin-rpy (cadr (massoc origin-sxml '@ 'rpy))))
    (make-coords :pos (if origin-xyz
                          (scale scale
                                 (read-float-vector-from-xml-value origin-xyz))
                        (float-vector 0 0 0))
                 :rpy (if origin-rpy
                          (read-float-vector-from-xml-value origin-rpy)
                        (float-vector 0 0 0)))))


(defun make-urdf-materials (robot-sxml)
  (let ((materials (remove-if-not #'(lambda (x) (eq (car x) 'material))
                                  (cdr robot-sxml))))
    (remove-if
     #'null
     (mapcar
      #'(lambda (material)
          (if (assoc 'texture material)
              (warn "texture not supported~%")
            (let ((ret (instance colormaterial :create
                                 :name
                                 (cadr (massoc material '@ 'name))
                                 :diffuse
                                 (read-float-vector-from-xml-value
                                  (cadr (massoc material 'color '@ 'rgba))))))
              (cons (cadr (massoc material '@ 'name)) ret))))
      materials))))

(defun make-urdf-body (sxml materials &key (scale 1000.0))
  (let ((attr (sxml-attribute sxml))
        (contents (cddr sxml)))
    (let ((link-name (cadr (assoc 'name (cdr attr)))))
      (let ((visual-sxml (cdr (assoc 'visual contents))))
        (let ((origin-sxml (assoc 'origin visual-sxml))
              (geometry-sxml (assoc 'geometry visual-sxml))
              (material-sxml (assoc 'material visual-sxml)))
          (when (and geometry-sxml origin-sxml)
            (let ((origin-coords (make-origin-coords origin-sxml)))
              (let ((ret (make-geometry-from-urdf geometry-sxml
                                                  :name link-name
                                                  :coords origin-coords
                                                  :scale scale)))
                (let* ((material-name (cadr (massoc material-sxml '@ 'name)))
                       (material (cdr (assoc material-name materials
                                             :test #'string=))))
                  (setf (get ret :face-color) material))
                ret))))))))

;;;
;;;
(defun make-jointattr-list-from-robot-urdf (sxml bodies &key (scale 1000))
  (let ((joints-sxml
         (remove-if-not #'(lambda (x) (eq (car x) 'joint)) sxml))
        jointattr-list)
    (dolist (joint-sxml joints-sxml)
      (let (joint
            (joint-name (cadr (massoc joint-sxml '@ 'name)))
            (joint-type (cadr (massoc joint-sxml '@ 'type))))
        (let ((origin-coords (make-origin-coords (assoc 'origin joint-sxml)))
              (parent-link-name (cadr (massoc joint-sxml 'parent '@ 'link)))
              (child-link-name (cadr (massoc joint-sxml 'child '@ 'link))))
          (when (and parent-link-name child-link-name)
            (let ((axis-sxml (massoc joint-sxml 'axis '@ 'xyz))
                  (low-limit-sxml (massoc joint-sxml 'limit '@ 'lower))
                  (upper-limit-sxml (massoc joint-sxml 'limit '@ 'upper)))
              (let ((axis (if axis-sxml (read-float-vector-from-xml-value
                                         (cadr axis-sxml))))
                    (low-limit (if low-limit-sxml
                                   (read-from-string (cadr low-limit-sxml))))
                    (upper-limit (if upper-limit-sxml
                                     (read-from-string (cadr upper-limit-sxml)))))
                (push (list (cons :name joint-name) (cons :type joint-type)
                            (cons :coords origin-coords)
                            (cons :axis axis)
                            (cons :min low-limit) (cons :max upper-limit)
                            (cons :parent-link-name parent-link-name)
                            (cons :child-link-name child-link-name)) jointattr-list)
                ))))))
    (nreverse jointattr-list)))

(defun make-body-list-from-robot-urdf (sxml &key (scale 1000))
  (let ((bodies-sxml
         (remove-if-not #'(lambda (x) (eq (car x) 'link)) sxml))
        (materials (make-urdf-materials sxml)))
    (mapcan
     #'(lambda (x)
         (let ((b (make-urdf-body x materials :scale scale)))
           (if b (list b))))
     bodies-sxml)))

(defun make-bodyset-bodies-from-jointattr (abody jointattr-list body-list)
  ;; search joint that has link-name as parent
  (append (list abody)
          (mapcan
           #'(lambda (jointattr)
               (if (and (string= (cdr (assoc :parent-link-name jointattr)) (send abody :name))
                        (string= (cdr (assoc :type jointattr)) "fixed"))
                   (let ((newbody (find-if #'(lambda (b)
                                               (string= (send b :name)
                                                        (cdr (assoc :child-link-name jointattr)))) body-list))
                         (coords (send (cdr (assoc :coords jointattr)) :copy-worldcoords))
                         )
                     (when newbody
                       (send newbody :move-to (send abody :copy-worldcoords) :world)
                       (send abody :assoc newbody)
                       (send newbody :move-to coords :parent)
                       (make-bodyset-bodies-from-jointattr
                        newbody jointattr-list body-list)))))
           jointattr-list)))

;;;
;;;
;;;
(defun make-bodyset-link-list-from-jointattr (jointattr-list body-list)
  (let (bodyset-list)
    (dolist (jointattr jointattr-list)
      (let ((joint-type (cdr (assoc :type jointattr)))
            (child-link-name (cdr (assoc :child-link-name jointattr)))
            (parent-link-name (cdr (assoc :parent-link-name jointattr)))
            (coords (cdr (assoc :coords jointattr)))
            bodies abody)
        (when (or (string= joint-type "prismatic")
                  (string= joint-type "revolute")
                  (string= joint-type "continuous"))
          (setq abody
                (find-if #'(lambda (x) (string= (send x :name) child-link-name)) body-list))
          (setq bodies (make-bodyset-bodies-from-jointattr abody jointattr-list body-list))
          (push
           (instance bodyset-link :init (make-cascoords) :bodies bodies :name child-link-name)
           bodyset-list)
          )))
    (nreverse bodyset-list)))

(defun make-joint-list-from-jointattr (jointattr-list link-list &key (scale 1000))
  (let (joint-list)
    (dolist (jointattr jointattr-list)
      (let ((joint-type (cdr (assoc :type jointattr)))
            (joint-name (cdr (assoc :name jointattr))))
        (when (member joint-type (list "revolute" "continuous" "prismatic") :test #'string=)
          (let ((child-link
                 (find-if #'(lambda (x) (member (cdr (assoc :child-link-name jointattr)) (send-all (send x :bodies) :name)
                                                :test #'string=)) link-list))
                (parent-link
                 (find-if #'(lambda (x) (member (cdr (assoc :parent-link-name jointattr)) (send-all (send x :bodies) :name)
                                                :test #'string=)) link-list))
                (min (cdr (assoc :min jointattr)))
                (max (cdr (assoc :max jointattr)))
                (axis (cdr (assoc :axis jointattr)))
                (coords (cdr (assoc :coords jointattr))) ajoint)
            (when (and parent-link child-link)
              (send child-link :move-to (send parent-link :copy-worldcoords) :world)
              (send parent-link :assoc child-link)
              (send child-link :transform coords)
              (setq ajoint
                    (cond ((string= "revolute" joint-type)
                           (instance rotational-joint :init
                                     :min (rad2deg min) :max (rad2deg max)
                                     :parent-link parent-link :child-link child-link
                                     :name joint-name
                                     :axis axis))
                          ((string= "continuous" joint-type)
                           (instance rotational-joint :init
                                     :min -360 :max 360
                                     :parent-link parent-link :child-link child-link
                                     :name joint-name
                                     :axis axis))
                          ((string= "prismatic" joint-type)
                           (instance linear-joint :init
                                     :min (* scale min) :max (* scale max)
                                     :parent-link parent-link :child-link child-link
                                     :name joint-name
                                     :axis axis))))
              (if ajoint (push ajoint joint-list)))))))
    joint-list))

(defun make-robot-model-from-robot-urdf (robot-sxml &key (scale 1000.0))
  ;; simple error check
  (if (not (eq (car robot-sxml) 'robot))
      (error "robot-sxml must be start with robot"))
  (let* ((attr (cadr robot-sxml))
         (contents (cddr robot-sxml))
         (bodies-sxml (remove-if-not #'(lambda (x) (eq (car x) 'link)) contents))
         (body-list (make-body-list-from-robot-urdf contents :scale scale))
         (jointattr-list (make-jointattr-list-from-robot-urdf contents body-list :scale scale))
         link-list joint-list
         )
    (format t "~A bodies~%" (length body-list))
    (format t "~A joints~%" (length jointattr-list))
    ;;
    (setq link-list (make-bodyset-link-list-from-jointattr jointattr-list body-list))
    ;; make root bodyset
    (let ((root-name
           (car (remove-if #'(lambda (x)
                               ;; any body-names includes in joint-list
                               (let ((child-link-names
                                      (mapcar #'(lambda (x) (cdr (assoc :child-link-name x))) jointattr-list)))
                                 (car (member x child-link-names :test #'string=))))
                           (mapcar #'(lambda (x) (cadr (massoc x '@ 'name))) bodies-sxml)))))
      (push
       (instance bodyset-link :init (make-cascoords)
                 :name root-name
                 :bodies (make-bodyset-bodies-from-jointattr (find-if #'(lambda (x) (string= (send x :name) root-name)) body-list) jointattr-list body-list))
       link-list))
    ;;
    (setq joint-list (make-joint-list-from-jointattr jointattr-list link-list :scale scale))
    (list link-list joint-list)))

(defclass urdf-robot
  :super robot-model
  :slots ())
(defmethod urdf-robot
  (:init
    (fname &rest args)
    (let ()
      (send-super* :init args)
      (multiple-value-setq (links joint-list)
                           (car (urdf2eus fname)))

      (setq larm (mapcan #'(lambda (x) (if (and (string= "l_" (subseq (send x :name) 0 2))
                                                (not (string= "l_gripper" (subseq (send x :name) 0 9))))
                                           (list x))) links))
      (setq rarm (mapcan #'(lambda (x) (if (and (string= "r_" (subseq (send x :name) 0 2))
                                                (not (string= "r_gripper" (subseq (send x :name) 0 9))))
                                           (list x))) links))
      (setq head (mapcan #'(lambda (x) (if (string= "head_" (subseq (send x :name) 0 5)) (list x))) links))
      (setq torso (mapcan #'(lambda (x) (if (string= "torso_" (subseq (send x :name) 0 6)) (list x))) links))

      (setq larm-root-link (car larm)
            rarm-root-link (car rarm)
            head-root-link (car head)
            torso-root-link (car torso)
            larm-end-coords (send (make-cascoords :coords (send (car (last larm)) :copy-worldcoords)) :translate #f(150 0 0))
            rarm-end-coords (send (make-cascoords :coords (send (car (last rarm)) :copy-worldcoords)) :translate #f(150 0 0))
            head-end-coords (send (send (make-cascoords :coords (send (car (last head)) :copy-worldcoords)) :translate #f(80 0 100)) :orient (/ pi 1.5) #f(-1 1 -1)))
      (send (car (last larm)) :assoc larm-end-coords)
      (send (car (last rarm)) :assoc rarm-end-coords)
      (send (car (last head)) :assoc head-end-coords)
      (dolist (l links)
        (send l :name (intern (format nil "~A" (string-upcase (send l :name))))))
      (dolist (j joint-list)
        (send j :name (intern (format nil "~A" (string-upcase (send j :name))))))

      (send self :init-ending)
      self)))
;;

(defun extract-robot-tag-from-urdf (sxml)
  ;; sxml starts with (*TOP* (*PI* ...) (content1 ...) (content2 ...))
  ;; skip *top* and *pi*
  (let ((contents sxml));;(find-if #'(lambda (x) (if (listp x)
    ;;                     (eq (car x) 'robot)))
    ;;   sxml)))
    (remove-if-not #'(lambda (x) (if (listp x) (eq (car x) 'robot)))
                   contents)))

(defun parse-urdf-sxml (sxml &key (scale 1000.0) (output))
  (let ((robots (extract-robot-tag-from-urdf sxml)))
    (mapcar #'(lambda (x)
                (make-robot-model-from-robot-urdf x :scale scale))
            robots)))

;;;
;;;

(defun urdf2eus (fname &key (scale 1000.0))
  (cond ((probe-file fname))
        ((probe-file (format nil "~A/models/~A" (ros::rospack-find "eusurdf") fname))
         (setq fname (format nil "~A/models/~A" (ros::rospack-find "eusurdf") fname)))
        (t
         (warn ";; WARN could not find urdf model file ~A~%" fname)
         (warn ";; type rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro -o full_pr2.urdf or type `roscd eusurdf`; make~%")))
  (let ((sxml (read (piped-fork (format nil "`rospack find eusurdf`/scripts/xml2sxml.scm ~A" fname)))))
    (parse-urdf-sxml sxml :scale scale :output (format nil "~A.l" (pathname-name fname)))))

;; (load "urdf2eus.l")
;; (setq *pr2* (instance urdf-robot :init "full_pr2.urdf"))
;; (send *pr2* :inverse-kinematics (send *pr2* :larm :end-coords) :move-target (send *pr2* :rarm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent) (send *pr2* :rarm :root-link)) :debug-view t)
