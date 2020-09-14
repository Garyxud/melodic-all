(defvar *use-assimp* t)
(defvar *remove-worldlink* t)
(defvar *remove-fixed-joint* nil)
;;(defvar *default-direction* :-y-up)
(defvar *default-direction* :z-up)
(defvar *plane-scale* 1000.0)

(unless (boundp '*worldcoords*) (setq *worldcoords* (make-coords)))
(unless (fboundp 'stl2eus) (load "read-stl.l"))
(when (and (boundp '*use-assimp*) *use-assimp*)
  (load "package://eus_assimp/euslisp/eus-assimp.l"))
(load "package://euscollada/src/xml.l")
(defvar *xml2sxml* (format nil "~A/scripts/xml2sxml.scm" (ros::rospack-find "eusurdf")))

(defun elt_x (v) (elt v 0))
(defun elt_y (v) (elt v 1))
(defun elt_z (v) (elt v 2))
(defun convert-minus-to-underbar (str)
  (cond
   ((symbolp str) (setq str (symbol-string str)))
   ((not (stringp str))
    (return str)))
  (while t
    (let ((p (position #\- str)))
      (if p (setf (elt str p) #\_)
        (return str)))))

(defun massoc (alist &rest args)
  (if (null args)
      alist
    (apply #'massoc (assoc (car args) alist) (cdr args))))

(defun read-sxml (fname)
  (let ((strm (piped-fork (format nil "~A ~A" *xml2sxml* fname))))
    (unwind-protect
        (read strm)
      (close strm))))

(defun urdf2eus (fname &key (scale 1000.0))
  (let ((sxml (read-sxml fname)))
    (parse-urdf-sxml sxml :scale scale)))
(defun sdf2eus (fname &key (scale 1000.0))
  (let ((sxml (read-sxml fname)))
    (parse-sdf-sxml sxml :scale scale)))

(defun parse-urdf-sxml (sxml &key (scale 1000.0))
  (let ((robots (extract-robot-tag-from-urdf sxml)))
    (mapcar #'(lambda (x)
                (make-robot-model-from-robot-urdf x :scale scale))
            robots)))
(defun parse-sdf-sxml (sxml &key (scale 1000.0))
  (let ((sdfmodel (extract-model-tag-from-sdf sxml)))
    (make-robot-model-from-robot-urdf sdfmodel :scale scale :robot 'model)))

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

(defun make-robot-model-from-robot-urdf (robot-sxml &key (scale 1000.0) (robot 'robot))
  ;; simple error check
  (if (not (eq (car robot-sxml) robot))
      (error "robot-sxml must be start with ~A" robot))
  (let ((attr (cadr robot-sxml))
        (contents (cddr robot-sxml)))
    (let ((links-sxml
           (remove-if-not #'(lambda (x) (eq (car x) 'link)) contents))
          (joints-sxml
           (remove-if-not #'(lambda (x) (eq (car x) 'joint)) contents)))
      (format t "~A links~%" (length links-sxml))
      (format t "~A joints~%" (length joints-sxml))
      (let ((materials (make-urdf-materials robot-sxml)))
        (let ((links (mapcar
                      #'(lambda (x)
                          (make-urdf-link x materials :scale scale))
                      links-sxml))
              rawlinks)

          (let ((joints (mapcar #'(lambda (x)
                                    (make-urdf-joint x links :scale scale))
                                joints-sxml)))
            (setq links (remove-if #'null links))
            (setq joints (remove-if #'null joints))
            (when *remove-worldlink*
              (let ((wlk (find-if #'(lambda (x) (string= (get x :name) "world")) links)))
                (when wlk
                  (when (send wlk :parent)
                    ;; ???
                    (send (send wlk :parent) :dissoc wlk))
                  (dolist (d (send wlk :descendants)) (send wlk :dissoc d))
                  (dolist (cl (send wlk :child-links))
                    (setq (cl . parent-link) nil))
                  (setq links (remove wlk links)))
                (let ((wj (find-if #'(lambda (x) (eq (send x :parent-link) *worldcoords*)) joints)))
                  (when wj
                    (send (send wj :child-link) :del-joint)
                    (setq joints (remove wj joints)))
                  )))
            (setq rawlinks links)
            (when *remove-fixed-joint*
              (let ((fixed-j (remove-if-not #'(lambda (x) (get x :fixed-joint)) joints))
                    fixed-l)
                (dolist (fj fixed-j)
                  (push (send fj :child-link) fixed-l))
                (setq links (set-difference links fixed-l))
                (setq joints (set-difference joints fixed-j))
                ))
            (let ((ret (instance cascaded-link)))
              ;; hack!
              (setq (ret . links) links)
              (setq (ret . joint-list) joints)
              (send ret :init :name (cadr (massoc robot-sxml '@ 'name)))
              ;; search root link
              (let ((rt (find-if #'(lambda (x) (null (send x :parent))) rawlinks)))
                (if rt (send ret :assoc rt)
                  (warn ";; can not find root link?~%")))
              (send ret :init-ending)
              (setq (ret . bodies) (remove-if #'null (flatten (send-all rawlinks :bodies))))
              (unless (eq rawlinks links)
                (setf (get ret :rawlinks) rawlinks))
              ret)))))))

(defun make-origin-coords (origin-sxml &key (scale 1000.0))
  (cond
   ((eq 'pose (car origin-sxml))
    (let* ((vec (read-float-vector-from-xml-value (cadr origin-sxml)))
           (pos (subseq vec 0 3))
           (rpy (subseq vec 3)))
      (scale scale pos pos)
      (make-coords :pos pos :rpy (list (elt rpy 2) (elt rpy 1) (elt rpy 0)))
      ))
   (t
    (make-origin-coords-org origin-sxml :scale scale)
    )))
(defun make-origin-coords-org (origin-sxml &key (scale 1000.0))
  (let ((origin-xyz (cadr (massoc origin-sxml '@ 'xyz)))
        (origin-rpy (cadr (massoc origin-sxml '@ 'rpy))))
    (make-coords :pos (if origin-xyz
                          (scale scale
                                 (read-float-vector-from-xml-value origin-xyz))
                        (float-vector 0 0 0))
                 :rpy (if origin-rpy
                          (let ((rpy (read-float-vector-from-xml-value origin-rpy)))
                            (list (elt rpy 2) (elt rpy 1) (elt rpy 0)))
                        (float-vector 0 0 0)))))

(defun rearrange-urdf-link (parent-link child-link origin-coords)
  (let ((org-coords (send child-link :copy-coords)))
    (send child-link :newcoords (send parent-link :copy-worldcoords))
    (send child-link :worldpos)
    (unless (and *remove-worldlink*
                 (eq parent-link *worldcoords*))
      (send parent-link :assoc child-link))
    (send child-link :transform origin-coords)
    (send child-link :transform org-coords)))

(defun _make-urdf-joint (sxml joint-type parent-link child-link joint-name
                              scale)
  (let ((axis-sxml (or (massoc sxml 'axis '@ 'xyz)
                       (massoc sxml 'axis 'xyz)))
        (low-limit-sxml (or (massoc sxml 'limit '@ 'lower)
                            (massoc sxml 'axis 'limit 'lower)))
        (upper-limit-sxml (or (massoc sxml 'limit '@ 'upper)
                              (massoc sxml 'axis 'limit 'upper))))
    (let ((axis (if axis-sxml (read-float-vector-from-xml-value
                               (cadr axis-sxml))))
          (low-limit (if low-limit-sxml
                         (read-from-string (cadr low-limit-sxml))))
          (upper-limit (if upper-limit-sxml
                           (read-from-string (cadr upper-limit-sxml)))))
      (cond
       ((string= "revolute" joint-type)
         (instance rotational-joint :init
                   :min (if low-limit (rad2deg low-limit) most-negative-float)
                   :max (if upper-limit (rad2deg upper-limit) most-positive-float)
                   :parent-link parent-link :child-link child-link
                   :name joint-name
                   :axis axis))
       ((string= "continuous" joint-type)
        (instance rotational-joint :init
                  :parent-link parent-link :child-link child-link
                  :name joint-name
                  :axis axis))
       ((string= "prismatic" joint-type)
         (instance linear-joint :init
                   :min (if low-limit (* scale low-limit) most-negative-float)
                   :max (if upper-limit (* scale upper-limit) most-positive-float)
                   :parent-link parent-link :child-link child-link
                   :name joint-name
                   :axis axis))
       ((string= "floating" joint-type)
         (instance 6dof-joint :init
                   :parent-link parent-link :child-link child-link
                   :name joint-name))
       ((string= "planar" joint-type)
        (warn "sorry planar joint does not supported~%")
        (instance rotational-joint :init
                  :parent-link parent-link :child-link child-link
                  :min 0 :max 0
                  :name joint-name))
       ((string= "fixed" joint-type)
        (let ((ret
               (instance rotational-joint :init
                         :parent-link parent-link :child-link child-link
                         :min 0 :max 0
                         :name joint-name)))
          (setf (get ret :fixed-joint) t)
          ret))
       (t
        (warn "~A is not supported~%" joint-type))))))

(defun make-urdf-joint (sxml links &key (scale 1000.0))
  (let ((joint-name (cadr (massoc sxml '@ 'name)))
        (joint-type (cadr (massoc sxml '@ 'type))))
    (let ((origin-coords (make-origin-coords (or (assoc 'origin sxml)
                                                 (assoc 'pose sxml))))
          (parent-link-name (or (cadr (massoc sxml 'parent '@ 'link))
                                (cadr (assoc 'parent sxml))))
          (child-link-name (or (cadr (massoc sxml 'child '@ 'link))
                               (cadr (assoc 'child sxml)))))
      (when (and parent-link-name child-link-name)
        (let ((parent-link (if (and *remove-worldlink*
                                    (string= parent-link-name "world"))
                               *worldcoords*
                             (find-if
                              #'(lambda (x) (string= (get x :name)
                                                     parent-link-name))
                              links)))
              (child-link (find-if
                            #'(lambda (x) (string= (get x :name)
                                                   child-link-name))
                            links)))
          ;; resolve position
          ;; origin of child -> parent-link + origin-coords
          (rearrange-urdf-link parent-link child-link origin-coords)
          (_make-urdf-joint sxml joint-type
                            parent-link child-link joint-name scale)
          )))))

(defun sxml-attribute (sxml)
  (assoc '@ sxml))

(defun read-list-from-xml-value (str)
  (let ((input (make-string-input-stream str)))
    (let ((tokens nil)
          (token nil))
      (while (setq token (read input nil nil))
        (push token tokens))
      (reverse tokens))))

(defun read-float-vector-from-xml-value (str)
  (coerce (read-list-from-xml-value str) float-vector))

(defun make-geometry-from-urdf (sxml &key
                                     (coords (make-coords))
                                     (scale 1000.0))
  (let ((specifier (caar (remove-if
                          #'(lambda (x) (eq (car x) '@)) (cdr sxml))))
        (contents (cdar (remove-if
                         #'(lambda (x) (eq (car x) '@)) (cdr sxml)))))
    (when sxml
      (case specifier
        (mesh
         (let ((fname (or (cadr (massoc contents '@ 'filename))
                          (cadr (assoc 'uri contents)))))
           (let ((path (ros::resolve-ros-path fname))
                 ret)
             (cond
              ((and (boundp '*use-assimp*)
                   *use-assimp*)
               (setq ret (load-mesh-file path :scale scale :direction *default-direction*))
               (when ret
                 (send ret :transform coords)
                 ))
              (t
               (setq ret (stl2eus path :coords coords :scale scale))))
             ret
             )))
        (box
         (let ((val (mapcar #'(lambda (x) (* scale x))
                            (read-list-from-xml-value
                             (or (cadr (massoc contents '@ 'size))
                                 (cadr (assoc 'size contents)))))))
           (make-cube (elt val 0) (elt val 1) (elt val 2) :coords coords)))
        (cylinder
         (let ((radius (* scale (read-from-string
                                 (or
                                  (cadr (massoc contents '@ 'radius))
                                  (cadr (assoc 'radius contents))))))
               (length (* scale (read-from-string
                                 (or (cadr (massoc contents '@ 'length))
                                     (cadr (assoc 'length contents)))))))
           (let ((cyl (make-cylinder radius length)))
             (send cyl :translate-vertices (float-vector 0 0 (/ length -2.0)))
             (send cyl :transform coords)
             cyl)))
        (sphere
         (warn "spehre does not supported~%")
         (make-cube 1 1 1))
        (plane
         (let ((nm (if (assoc 'normal contents) (read-list-from-xml-value (cadr (assoc 'normal contents)))))
               (size (if (assoc 'size contents) (read-list-from-xml-value (cadr (assoc 'size contents))))))
           ;; not using normal
           (unless size (setq size (list 1000 1000)))
           (let ((cb
                  (make-cube (* *plane-scale* (car size)) (* *plane-scale* (cadr size)) 10)))
             (send cb :translate-vertices (float-vector 0 0 -5))
             cb)))
        (heightmap ;; for sdf
         ;; not implemented yet
         (make-cube 10 10 10)
         )
        (t
         (error "~A is not supported" sxml))))))

(defun make-urdf-link-single (sxml materials &key (scale 1000.0))
  (let ((attr (sxml-attribute sxml))
        (contents (cddr sxml)))
    (let ((link-name (cadr (assoc 'name (cdr attr)))))
      (let ((visual-sxml
             (cdr (assoc 'visual contents)))
            (collision-sxml
             (cdr (assoc 'collision contents)))
            vis-geom col-geom)
        ;; visual
        (let ((origin-sxml (assoc 'origin visual-sxml))
              (geometry-sxml (assoc 'geometry visual-sxml))
              (material-sxml (assoc 'material visual-sxml)))
          (let ((origin-coords (make-origin-coords origin-sxml)))
            (let ((ret (make-geometry-from-urdf geometry-sxml
                                                :coords origin-coords
                                                :scale scale)))
              (let* ((material-name
                      (cadr (massoc material-sxml '@ 'name)))
                     (material (cdr (assoc material-name materials
                                           :test #'string=))))
                (when (and ret material)
                  (setf (get ret :face-color) material))
                (setq vis-geom ret)
                ))))
        ;; collision
        (let ((origin-sxml (assoc 'origin collision-sxml))
              (geometry-sxml (assoc 'geometry collision-sxml)))
          (let ((origin-coords (make-origin-coords origin-sxml)))
            (let ((ret (make-geometry-from-urdf geometry-sxml
                                                :coords origin-coords
                                                :scale scale)))
              (setq col-geom ret)
              )))
        (let ((ret (instance bodyset-link :init (make-cascoords)
                             :bodies (list (or vis-geom (make-cube 5 5 5)))
                             :name link-name)))
          (setf (get ret :collision-geometory) col-geom)
          ret)))))
;; for multi collision/visual
(defun parse-sdf-material (obj material-sxml) obj) ;; not implemented yet
(defun make-urdf-link (sxml materials &key (scale 1000.0))
  (let ((attr (sxml-attribute sxml))
        (contents (cddr sxml)))
    (let ((link-name (cadr (assoc 'name (cdr attr))))
          (link-pose (assoc 'pose contents))
          (link-inertial (assoc 'inertial contents))
          vis-geom col-geom)
      (when link-pose
        (setq link-pose (make-origin-coords link-pose)))
      (block :extract-visual
        (while t
          (let ((visual-sxml
                 (assoc 'visual contents))
                (collision-sxml
                 (assoc 'collision contents)))
            (unless (or visual-sxml collision-sxml)
              (return-from :extract-visual))
            (setq contents (delete visual-sxml contents))
            (setq contents (delete collision-sxml contents))
            (setq visual-sxml (cdr visual-sxml))
            (setq collision-sxml (cdr collision-sxml))
            ;; visual
            (when visual-sxml
              (let ((origin-sxml (or (assoc 'origin visual-sxml)
                                     (assoc 'pose visual-sxml)))
                    (geometry-sxml (assoc 'geometry visual-sxml))
                    (material-sxml (assoc 'material visual-sxml)))
                (let ((origin-coords (make-origin-coords origin-sxml)))
                  (let ((ret (make-geometry-from-urdf geometry-sxml
                                                      :coords origin-coords
                                                      :scale scale)))
                    (let* ((material-name
                            (cadr (massoc material-sxml '@ 'name)))
                           (material (cdr (assoc material-name materials
                                                 :test #'string=))))
                      (unless material (setq ret (parse-sdf-material ret material-sxml)))
                      (when ret
                        (if material (setf (get ret :face-color) material))
                        (push ret vis-geom))
                      )))))
            ;; collision
            (when collision-sxml
              (let ((origin-sxml (or (assoc 'origin visual-sxml)
                                     (assoc 'pose visual-sxml)))
                    (geometry-sxml (assoc 'geometry collision-sxml)))
                (let ((origin-coords (make-origin-coords origin-sxml)))
                  (let ((ret (make-geometry-from-urdf geometry-sxml
                                                      :coords origin-coords
                                                      :scale scale)))
                    (when ret (push ret col-geom))
                    ))))
            )));; :extract-visual
      (when vis-geom
        (let ((p (car vis-geom)))
          (dolist (c (cdr vis-geom))
            (send p :assoc c))))
      (let ((ret (instance bodyset-link :init (make-cascoords)
                           :bodies (or vis-geom (list (make-cube 5 5 5)))
                           :name link-name)))
        (setf (get ret :collision-geometory) col-geom)
        (when link-inertial
          (set-inertial-param-to-link ret link-inertial))
        (when link-pose
          (send ret :transform link-pose))
        ret))))
(defun set-inertial-param-to-link (blink inertial)
  (let ((ms (cadr (massoc inertial 'mass '@ 'value)))
        (xyz (cadr (massoc inertial 'origin '@ 'xyz)))
        ;;(rpy (cadr (massoc inertial 'origin '@ 'rpy)))
        (inn (cdr (massoc inertial 'inertia '@))))
    (when ms
      (send blink :weight (* 1000.0 (read-from-string ms))))
    (when xyz
      (setq (blink . acentroid)
            (scale 1000.0 (read-float-vector-from-xml-value xyz))))
    (when inn
      (let ((xx (assoc 'ixx inn))
            (yy (assoc 'iyy inn))
            (zz (assoc 'izz inn))
            (xy (assoc 'ixy inn))
            (xz (assoc 'ixz inn))
            (yz (assoc 'iyz inn))
            (mat (make-matrix 3 3)))
        (when xx
          (setf (aref mat 0 0) (read-from-string (cadr xx))))
        (when yy
          (setf (aref mat 1 1) (read-from-string (cadr yy))))
        (when zz
          (setf (aref mat 2 2) (read-from-string (cadr zz))))
        (when xy
          (let ((val (read-from-string (cadr xy))))
            (setf (aref mat 0 1) val)
            (setf (aref mat 1 0) val)))
        (when xz
          (let ((val (read-from-string (cadr xz))))
            (setf (aref mat 0 2) val)
            (setf (aref mat 2 0) val)))
        (when yz
          (let ((val (read-from-string (cadr yz))))
            (setf (aref mat 1 2) val)
            (setf (aref mat 2 1) val)))
        (send blink :inertia-tensor mat)
        ))
    ))
(defun extract-robot-tag-from-urdf (sxml)
  ;; sxml starts with (*TOP* (*PI* ...) (content1 ...) (content2 ...))
  ;; skip *top* and *pi*
  (let ((contents sxml))
    (remove-if-not #'(lambda (x) (if (listp x) (eq (car x) 'robot)))
                   contents)))
(defun extract-model-tag-from-sdf (sxml)
  ;; sxml starts with (*TOP* (*PI* ...) (content1 ...) (content2 ...))
  ;; skip *top* and *pi*
  (let ((contents sxml) ret)
    (if (eq (car contents) '*top*)
        (setq contents (cdr contents)))
    (if (setq ret (assoc 'model contents))
        ret
      (dolist (ct contents)
        (if (listp ct)
            (setq ret (extract-model-tag-from-sdf ct)))
        (if ret (return-from extract-model-tag-from-sdf ret))
        ))))

(defun get-all-child-links (parent)
  (if (null parent)
      nil
    (append (list parent)
            (mapcar #'get-all-child-links (send parent :descendants)))))

(defmethod cascaded-link
  (:box (&rest args)
    (make-bounding-box (flatten bodies))))

(defmethod bodyset-link
  (:box (&rest args)
    (make-bounding-box (flatten bodies))))

(defun setup-urdf-filesystem (obj-name base-dir)
  (if (not (eq (probe-file base-dir) :directory))
      (error "~A must be a directory" base-dir)
    (progn
      (unix::mkdir (format nil "~A/meshes" base-dir))
      (unix::mkdir (format nil "~A/meshes/~A" base-dir obj-name))
      (unix::mkdir (format nil "~A/urdf" base-dir))
      (unix::mkdir (format nil "~A/launch" base-dir))
      (unix::mkdir (format nil "~A/config" base-dir))
      (unix::mkdir (format nil "~A/rviz" base-dir)))))

(defun eus2urdf (obj base-dir base-dir-package-name
                 &key 
                 (scale 0.001)
                 (append-links nil)
                 (one-mesh nil)
                 (launch t)
                 (write-robot-name t)
                 (prefix-name "")
                 (file-name nil)
                 (file-suffix ".urdf")
                 (parent-frame "/map")
                 (namespace))
  "convert from eus model to urdf file.
arguments:
 obj -- an instance of eus
 base-dir -- urdf output directory. `eus2urdf' generate a file system like
             base-dir/
                   +----meshes/
                   +----urdf/
 base-dir-package-name -- the path of urdf must be resolvable with ros 
                          filesystem convention package://. 
                          base-dir-package-name specify `base-dir' path 
                          with package:// convention.

example:
  (eus2urdf (make-askul-1200x700-desk) 
    \"~/ros/boxturtle/jsk-ros-pkg/jsk_object_models\" \"jsk_object_models\")
"
  (setup-urdf-filesystem (gen-rvizable-name prefix-name
                                            (string (send obj :name)))
                         (namestring (truename base-dir)))
  (let ((name (send obj :name)))
    (if name
        (let ((prev-coodes (send obj :copy-worldcoords)))
          (unwind-protect
              (progn
                (send obj :move-to (make-coords) :world)
                (send obj :worldpos)
                (send-all (send obj :links) :worldpos)
                (convert-to-urdf obj (namestring (truename base-dir))
                                 base-dir-package-name
                                 prefix-name
                                 :file-name file-name
                                 :file-suffix file-suffix
                                 :write-robot-name write-robot-name
                                 :append-links append-links
                                 :scale scale
                                 :launch launch
                                 :one-mesh one-mesh
                                 :parent-frame parent-frame
                                 :namespace namespace))
            (send obj :move-to prev-coodes :world)
            (send obj :worldpos)
            (send-all (send obj :links) :worldpos)))
      (progn
        (warn "no name is specified: ~A~%" obj)
        nil))))

(defun non-jointed-links (links joints)
  (let ((connected-links (remove-duplicates
                          (append (send-all joints :parent-link)
                                  (send-all joints :child-link)))))
    (if joints
        (remove-if #'(lambda (x) (member x connected-links)) links)
      nil)))

(defun remove-non-jointed-links (links joints)
  (let ((connected-links (remove-duplicates
                          (append (send-all joints :parent-link)
                                  (send-all joints :child-link)))))
    (if joints
        (remove-if-not #'(lambda (x)
                           (member x connected-links))
                       links)
      links)))

(defun find-root-links (links joints)
  (let ((parents (remove-duplicates (send-all joints :parent-link)))
        (children (remove-duplicates (send-all joints :child-link))))
    (set-difference parents children)))

(defun find-root-link (links joints)
  (let ((parents (remove-duplicates (send-all joints :parent-link)))
        (children (remove-duplicates (send-all joints :child-link))))
    (car (set-difference parents children))))

(defun convert-one-root-tree-model-to-urdf (obj fname name 
                                            links joints
                                            parent-frame non-jointed-links
                                            base-dir base-dir-package-name
                                            scale write-robot-name launch
                                            &key (namespace))
  (let ((link-specs (mapcar #'dump-urdf-link-spec 
                            (append links non-jointed-links)))
        ;;(setf (get x :robot-model) obj)
        (joint-specs (mapcar #'(lambda (x)
                                 (let ((index (and (find-method x :servo)
                                                   (position (send x :servo)
                                                             (send obj :actuators)))))
                                   (if index
                                       (setf (get x :max-effort)
                                             (elt (send obj :max-torque-vector) index))
                                     (setf (get x :max-effort) 0)))
                                 (dump-urdf-joint-spec
                                  x
                                  (append 
                                   (if (and (not (derivedp obj robot-model))
                                            (find-method obj :larm))
                                       (send obj :larm :links))
                                   (if (and (not (derivedp obj robot-model))
                                            (find-method obj :lleg))
                                       (send obj :lleg :links)))
                                  (append 
                                   (if (and (not (derivedp obj robot-model))
                                            (find-method obj :rarm))
                                       (send obj :rleg :links))
                                   (if (and (not (derivedp obj robot-model))
                                            (find-method obj :rleg))
                                       (send obj :rarm :links)))))
                             joints))
        (fixed-joint-specs (mapcar #'(lambda (x)
                                       (dump-urdf-fix-joint-spec
                                        x 
                                        (find-root-link links joints)))
                                   non-jointed-links)))
    (with-open-file (f fname :direction :output)
      (write-urdf-header f name
                         :write-robot-name write-robot-name)
      (dolist (l link-specs)
        (write-urdf-link f l base-dir 
                         base-dir-package-name
                         name :scale scale
                         :namespace namespace))
      (dolist (j (append joint-specs fixed-joint-specs))
        (write-urdf-joint f j name :scale scale :namespace namespace))
      (write-urdf-footter f))
    (when launch
      (write-launch-file name links base-dir base-dir-package-name scale
                         parent-frame))))

(defun convert-model-as-bodies-to-urdf (obj fname name links parent-frame
                                        base-dir base-dir-package-name
                                        scale &key (namespace))
  (let ((links (send obj :bodies)))
    (let ((link-spec
           (list (cons :name name)
                 (cons :origin (make-coords))
                 (cons :mesh 
                       (instance faceset
                                 :init :faces
                                 (mapcan #'geo::face-to-triangle
                                         (flatten 
                                          (send-all links :faces)))))
                 (cons :color (urdf-color-material-from-link-material obj)))))
      (with-open-file (f fname :direction :output)
         (write-urdf-header f name)
         (write-urdf-link f link-spec base-dir 
                          base-dir-package-name
                          name :scale scale :namespace namespace)
         (write-urdf-footter f))
      (let ((obj-name (send obj :name)))
        (convert-minus-to-underbar obj-name)
        (unwind-protect
            (progn
              (send obj :name name)
              (write-launch-file name (list obj)
                                 base-dir base-dir-package-name scale
                                 parent-frame))
          (send obj :name obj-name))))))

(defun convert-to-urdf (obj base-dir base-dir-package-name prefix-name
                        &key
                        (scale 0.001)
                        (one-mesh nil)
                        (append-links nil)
                        (file-name nil)
                        (launch t)
                        (file-suffix ".urdf")
                        (write-robot-name t)
                        (parent-frame "map")
                        (namespace))
  (let ((name (gen-rvizable-name prefix-name (string (send obj :name)))))
    (let ((fname (format nil "~A/urdf/~A~A" base-dir (or file-name name)
                         file-suffix)))
      (let ((all-links (append (send obj :links) append-links)))
        (let* ((links (remove-non-jointed-links all-links
                                                (send obj :joint-list)))
               (joints (send obj :joint-list))
               (non-jointed-links (non-jointed-links all-links joints)))
          (send obj :angle-vector         ;send #f(0 0 0 ...)
                (instantiate float-vector (length (send obj :angle-vector))))
          (send obj :worldpos)            ;update
          (if (not one-mesh)
              (convert-one-root-tree-model-to-urdf 
               obj fname name links
               joints parent-frame non-jointed-links
               base-dir base-dir-package-name scale write-robot-name launch
               :namespace namespace)
            (progn
              (warn "dump as a one link~%")
              (convert-model-as-bodies-to-urdf 
               obj fname name links parent-frame
               base-dir base-dir-package-name scale
               :namespace namespace)))
          fname)))))

(defun write-launch-file (name links base-dir base-dir-package-name 
                          scale parent-frame)
  (let ((fname (format nil "~A/launch/~A.launch" base-dir name)))
    (let ((sxml `(launch
                  (param
                   (@ (name ,name)
                      (textfile
                       ,(format nil "$(find ~A)/urdf/~A.urdf"
                                base-dir-package-name name))))
                  (node
                   (@ (pkg "easy_state_publisher")
                      (name ,(format nil "$(anon ~A_publisher)" name))
                      (type "easy_tf_publisher"))
                   (remap
                    (@ (from "/robot_description")
                       (to ,name)))
                  ))))
      (let ((xml-str (cat-normal (sxml->xml sxml) "")))
        (with-open-file (f fname :direction :output)
          (format f xml-str))))))

(defun urdf-origin-sxml (origin scale)
  `(origin
    (@
     (xyz
      ,(format nil "~A ~A ~A"
               (* scale (elt (send origin :worldpos) 0))
               (* scale (elt (send origin :worldpos) 1))
               (* scale (elt (send origin :worldpos) 2))))
     (rpy
      ,(format nil "~A ~A ~A"
               (elt (car (send origin :rpy-angle)) 2)
               (elt (car (send origin :rpy-angle)) 1)
               (elt (car (send origin :rpy-angle)) 0))))))

(defun write-urdf-link (f link-spec base-dir base-dir-package-name
                          robot-name
                          &key (scale 0.001) (namespace))
  (let ((sxml `(link
                (@
                 (name
                  ,(if namespace (format nil "~A~A"
                                         namespace (string (cdr (assoc :name link-spec))))
                     (format nil "~A" (string (cdr (assoc :name link-spec)))))))
                ;; visual
                ,@(when (or ;;(assoc :origin link-spec)
                        (assoc :mesh link-spec))
                    `((visual
                     ,(when (assoc :origin link-spec)
                        (let ((origin (cdr (assoc :origin link-spec))))
                          (urdf-origin-sxml origin scale)))
                     ,(when (assoc :mesh link-spec)
                        (let ((mesh-fname (format nil "~A/meshes/~A/~A.stl"
                                base-dir
                                robot-name
                                (string (cdr (assoc :name link-spec)))))
                              (package-path
                               (ros::rospack-find base-dir-package-name)))
                          ;; first of all, create mesh
                          (eus2stl mesh-fname
                                   (cdr (assoc :mesh link-spec)) :scale scale)
                          `(geometry
                            (mesh
                             (@
                              (filename
                               ,(format nil "package://~A~A"
                                        base-dir-package-name
                                        (subseq mesh-fname (length package-path)))
#|#+:jsk
                               ,(reg-replace ;JSKEUS
                                 package-path 
                                 (format nil "package://~A" 
                                         base-dir-package-name)
                                 mesh-fname)
#-:jsk
                               ,(format nil "file://~A" mesh-fname)|#
                               ))))))
                     ,(when (cdr (assoc :color link-spec))
                        (let ((rgb (cdr (assoc :color link-spec))))
                          `(material
                            ;; FIXME: GC may cause name conflicts...
                            (@ (name ,(string (gensym)))) ;unique name required
                            (color
                             (@
                              (rgba
                               ;; TODO: transparency not supported
                               ,(format nil "~A ~A ~A 1.0" ;alpha = 1.0
                                        (elt rgb 0)
                                        (elt rgb 1)
                                        (elt rgb 2)))))))))))
                ;; collision
                ,@(when (or ;;(assoc :origin link-spec)
                        (assoc :mesh link-spec))
                    `((collision
                     ,(when (assoc :origin link-spec)
                        (let ((origin (cdr (assoc :origin link-spec))))
                          (urdf-origin-sxml origin scale)))
                     ,(when (assoc :mesh link-spec)
                        (let ((mesh-fname (format nil "~A/meshes/~A/~A.stl"
                                base-dir
                                robot-name
                                (string (cdr (assoc :name link-spec)))))
                              (package-path
                               (ros::rospack-find base-dir-package-name)))
                          ;; first of all, create mesh
                          (eus2stl mesh-fname
                                   (cdr (assoc :mesh link-spec)) :scale scale)
                          `(geometry
                            (mesh
                             (@
                              (filename
                               ,(format nil "package://~A~A"
                                        base-dir-package-name
                                        (subseq mesh-fname (length package-path)))
#|#+:jsk
                               ,(reg-replace ;JSKEUS
                                 package-path 
                                 (format nil "package://~A" 
                                         base-dir-package-name)
                                 mesh-fname)
#-:jsk
                               ,(format nil "file://~A" mesh-fname)|#
                               ))))))))))))
    (format f (cat-normal (sxml->xml sxml) ""))))

(defun write-urdf-joint (f joint-spec name &key (scale 0.001) (namespace))
  (let ((axis (cdr (assoc :axis joint-spec)))
        (origin (cdr (assoc :origin joint-spec)))
        (limit (cdr (assoc :limit joint-spec))))
    (let ((sxml `(joint
                  (@ (name ,(cdr (assoc :name joint-spec)))
                     (type ,(cdr (assoc :type joint-spec))))
                  ,@(if axis
                       `((axis
                          (@ (xyz ,(format nil "~A ~A ~A"
                                           (elt axis 0)
                                           (elt axis 1)
                                           (elt axis 2))))))
                      nil)
                  ,(urdf-origin-sxml origin scale)
                  (parent
                   (@ (link
                       ,(if namespace (format nil "~A~A"
                                              namespace
                                              (string (cdr (assoc :parent joint-spec))))
                          (format nil "~A" (string (cdr (assoc :parent joint-spec))))))))
                  (child
                   (@ (link
                       ,(if namespace (format nil "~A~A"
                                              namespace
                                              (string (cdr (assoc :child joint-spec))))
                          (format nil "~A" (string (cdr (assoc :child joint-spec))))))))

                  ,@(if limit
                        `((limit
                           (@ (effort ,(cdr (assoc :effort limit)))
                              (velocity ,(cdr (assoc :velocity limit)))
                              (upper ,(cdr (assoc :upper limit)))
                              (lower ,(cdr (assoc :lower limit))))))))))
      (format f (cat-normal (sxml->xml sxml) "")))))

(defun write-urdf-header (f name &key (write-robot-name t))
  (if write-robot-name
      (format f "<?xml version=\"1.0\" ?>~%<robot name=\"~A\"> ~%" name)
    (format f "<?xml version=\"1.0\" ?>~%<robot> ~%")))

(defun write-urdf-footter (f)
  (format f "</robot>~%"))

(defun link-to-faceset (link)
  (cond
   ((or (get link :gl-vertices-bak)
        (get link :gl-vertices))
    (instance faceset
              :init :faces
              (mapcan
               #'(lambda (vv)
                   (cond 
                    ((assoc :color vv)
                     nil)
                    ((assoc :vertices vv)
                     (list (instance
                            face :init :vertices
                            (cdr (cdr (assoc :vertices vv))))))))
               (or (get link :gl-vertices-bak)
                   (get link :gl-vertices)))))
   ((and (send link :bodies)
         (find-if #'(lambda (b) (and (assoc 'glvertices (send b :slots))
                                     (b . glvertices))) (send link :bodies)))
    (let (bds)
      (dolist (b (send link :bodies))
        (if (and (assoc 'glvertices (send b :slots)) (b . glvertices))
            (let* ((g (b . glvertices))
                   (fs (send g :convert-to-faceset)))
              (send fs :transform (send (send link :copy-worldcoords) :inverse-transformation) :world)
              (push fs bds)
              )))
      (car bds) ;; why use last 1 faceset
      ))
   (t
    (send (instance faceset :init
                    :faces (send (geo::body-to-triangles link nil) :faces))
          :move-to (send (send link :copy-worldcoords)
                         :inverse-transformation)))
   ))

(defun concat-class-name (obj)
  (let ((class-name (string (send (class obj) :name)))
        (name (string (send obj :name))))
    (format nil "~A_~A" name class-name)))

(defun dump-urdf-objects-as-fixed-links (objects
                                         &key (roots nil))
  "returns a list like (link-specs joint-specs)."
  (if (null objects)
      (list nil nil)
    (let ((target (car objects))
          (rest (cdr objects)))
      (let ((target-parent (send target :parent)))
        (let ((link-spec (dump-urdf-link-spec
                          target :without-mesh t))
              (joint-spec (dump-urdf-joint-spec-as-fixed
                           (format nil "~A_~A_fixed"
                                   (convert-minus-to-underbar (string (send target-parent :name)))
                                   (convert-minus-to-underbar (concat-class-name target)))
                           target-parent target)))
          ;; next...
          (multiple-value-bind
              (_link-spec _joint-spec)
              (dump-urdf-objects-as-fixed-links
               (if (member target-parent roots)
                   rest
                 (cons target-parent rest))
               :roots (cons target roots))
            (list (cons link-spec _link-spec)
                  (cons joint-spec _joint-spec))))))))

(defun dump-urdf-link-spec (link &key (without-mesh nil))
  ;; origin always (make-coords)
  ;; geometry
  (list (cons :name
              (convert-minus-to-underbar
               (if (and (boundp 'sensor) (derivedp link sensor))
                   (concat-class-name link)
                 (send link :name))))
        (cons :origin (make-coords))
        (if without-mesh nil
          (let ((fs (link-to-faceset link))) (if fs (cons :mesh fs))))
        (cons :color (urdf-color-material-from-link-material link))))

(defun serialize-eus-joint-name (_joint llinks rlinks)
  (convert-minus-to-underbar
   (cond ((member (send _joint :child-link) llinks)
          (format nil "L_~A" (string (send _joint :name))))
         ((member (send _joint :child-link) rlinks)
          (format nil "R_~A" (string (send _joint :name))))
         (t
          (string (send _joint :name))))))

(defun dump-urdf-fix-joint-spec (target-link root-link)
  (list (cons :name (concatenate string
                                 "fixed_" (convert-minus-to-underbar
                                           (string (send target-link :name)))))
        (cons :type "fixed")
        (cons :origin (send root-link :transformation target-link))
        (cons :parent (convert-minus-to-underbar (send root-link :name)))
        (cons :child (convert-minus-to-underbar (send target-link :name)))))

(defun dump-urdf-joint-spec-as-fixed (name parent child)
  (list (cons :name name)
        (cons :type "fixed")
        (cons :origin (send parent :transformation child))
        (cons :parent (convert-minus-to-underbar (string (send parent :name))))
        (cons :child
              (convert-minus-to-underbar 
               (if (derivedp child sensor)
                   (concat-class-name child)
                 (string (send child :name)))))))

(defun dump-urdf-joint-spec (_joint llinks rlinks)
  ;; only supports revolute
  (let ((reverse-p
	 (or (and (member (send _joint :child-link) rlinks)
		  (let ((joint-name (string (send _joint :name))))
		    (or (= (elt joint-name (1- (length joint-name))) #\R)
			(= (elt joint-name (1- (length joint-name))) #\Y))))
	     (and (not (member (send _joint :child-link) llinks))
		  (let ((joint-name (string (send _joint :name))))
		    (= (elt joint-name (1- (length joint-name))) #\Y))))))
    (list (cons :name (serialize-eus-joint-name _joint llinks rlinks))
	  (cons :type "revolute")
	  (cons :origin (send (send _joint :parent-link)
			      :transformation
			      (send _joint :child-link)))
	  (cons :limit
		(list
		 (cons :lower (deg2rad (if reverse-p
					   (* -1 (send _joint :max-angle))
					 (send _joint :min-angle))))
		 (cons :upper (deg2rad (if reverse-p
					   (* -1 (send _joint :min-angle))
					 (send _joint :max-angle))))
		 (cons :effort (get _joint :max-effort)) ;; (send _joint :max-joint-torque)
		 (cons :velocity (if (find-method _joint :max-joint-velocity)
                                     (send _joint :max-joint-velocity) 0))))
          (cons :axis
                (scale (if reverse-p -1 1)
                       (case (_joint . axis)
                         (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                         (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                         (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                         (t (if (listp (_joint . axis))
                                ;; 6dof joint...
                                #f(0 0 0)
                                (_joint . axis))))))
	  (cons :parent (convert-minus-to-underbar (send (send _joint :parent-link) :name)))
	  (cons :child (convert-minus-to-underbar (send (send _joint :child-link) :name))))
    ))

(defun find-link-color (link)
  (if (find-method link :bodies)
      (let ((non-sorted-bodies
             (flatten (mapcar #'(lambda (x)
                                  (if (derivedp x bodyset)
                                      (send x :bodies)
                                    x))
                              (send link :bodies)))))
        (let ((bodies (sort (copy-list non-sorted-bodies)
                            #'>
                            #'(lambda (x)
                                (send x :volume)))))
          (find-if
           #'(lambda (x)
               (derivedp x colormaterial))
           (mapcar #'(lambda (x)
                       (if (derivedp x colormaterial) x (gl::find-color x)))
                   (mapcar #'(lambda (x) (get x :face-color)) bodies)))))))

(defun urdf-color-material-from-link-material (link)
  (if (find-link-color link)
      (send (find-link-color link) :diffuse)
    (float-vector 0.5 0.5 0.5)))

(defun write-set-tf-node (f name coords parent child
                             &key (scale 0.001) (freq 20))
  (let ((pos (send coords :pos))
	(rpy (car (rpy-angle (send coords :rot)))))
    (format f "<node pkg=\"dynamic_tf_publisher\" type=\"tf_set.py\" ")
    (format f "name=\"set_~a_tf\"~%" name)
    (format f "args=\"~a ~a ~a ~a ~a ~a ~a ~a ~a\" />~%"
	    (* scale (elt_x pos)) (* scale (elt_y pos)) (* scale (elt_z pos))
	    (elt_x rpy) (elt_y rpy) (elt_z rpy)
	    parent child
	    (/ 1000.0 freq))))

(defun write-room-object-tf (f room &key (scale 0.001) (pfx "jsk_"))
  (let ((objs (eusmap-convert-target-models room)))
    (let ((room-name
           (gen-rvizable-name pfx (string (send room :name)))))
      (dolist (ob objs)
	(let ((rname (gen-rvizable-name pfx (string (send ob :name)))))
	  (write-set-tf-node f rname (send ob :copy-worldcoords) room-name
                                (format nil "~a/~a" room-name rname)
                                :scale scale))))))

;; fram_id:
;;   mesh_name: model.stl
;;   color:
;;     r: 1.0
;;     g: 0.0
;;     b: 0.0
;;     a: 1.0
(defun write-room-model-frame-dictionary-yaml
  (mapmodel base-dir base-dir-package-name pfx)
  (let ((fname (format nil "~A/config/~A.yaml"
                       (namestring (truename base-dir))
                       (send mapmodel :name))))
    (with-open-file (f fname :direction :output)
      (format f "model_and_frames:~%")
      (let ((objs (eusmap-convert-target-models mapmodel))
            (room-name (gen-rvizable-name pfx (string (send mapmodel :name)))))
      (dolist (o objs)
        (let ((rname (gen-rvizable-name pfx (string (send o :name)))))
        (let ((stl-name (format nil "~A.stl" rname))
              (frame-name rname))
          (format f " ~A:~%" 
                  (gen-rvizable-name "jsk_" (string (send o :name))))
          (format f "  mesh_name: \"package://~A/meshes/~A/~A\"~%"
                  base-dir-package-name rname stl-name)
          (let ((color (urdf-color-material-from-link-material o)))
            (format f "  color:~%")
            (format f "   r: ~A~%" (elt color 0))
            (format f "   g: ~A~%" (elt color 1))
            (format f "   b: ~A~%" (elt color 2))
            (format f "   a: 1.0~%")))))))))

(defun write-room-model-frame-dictionary-launch
  (mapmodel base-dir base-dir-package-name prefix-name)
  (let ((fname (format nil "~A/launch/~A_mfd.launch"
                       (namestring (truename base-dir))
                       (send mapmodel :name))))
    (with-open-file (f fname :direction :output)
      (format f "<launch>~%")
      (format f "<node name=\"model_frame_dictionary\" ~&")
      (format f "      pkg=\"model_frame_dictionary\"~%")
      (format f "      clear_params=\"true\"~%")
      (format f "      type=\"model_frame_dictionary.py\">~%")
      (format f "  <rosparam file=\"$(find ~A)/config/~A.yaml\"~%"
              base-dir-package-name (string (send mapmodel :name)))
      (format f "            command=\"load\" cleanup=\"true\"/>~%")
      (format f "  <param name=\"tf_prefix\" value=\"~A\" />~%"
              (gen-rvizable-name "jsk_" 
                                 (string (send mapmodel :name))))
      (format f "</node>~%")
    (format f "</launch>~%"))
    ))
  

(defun write-room-object-tf-launch (mapmodel base-dir
						  &key (scale 0.001)
(offset (make-coords)) (map-frame "map") (pfx "jsk_"))
  (let ((fname (format nil "~A/launch/~A_tf.launch"
                       (namestring (truename base-dir))
                       (send mapmodel :name))))
    (with-open-file (f fname :direction :output)
      (format f "<launch>~%")
      (format f "<node name=\"tf_publisher\" pkg=\"dynamic_tf_publisher\"
 type=\"tf_publish.py\"/>~%")
      (write-set-tf-node f (send mapmodel :name) offset map-frame
			    (gen-rvizable-name pfx (send mapmodel :name)))
      (write-room-object-tf f mapmodel :scale scale :pfx pfx)
    (format f "</launch>~%"))
    ))

(defun eusmap2urdf (mapmodel base-dir base-dir-package-name 
                             &key (offset (make-coords)) (launch t))
  (let ((prefix-name (concatenate string "jsk_" (send mapmodel :name) "_")))
    ;; make urdf, stl and each launch file
    (mapcar #'(lambda (x) 
                (eus2urdf x base-dir base-dir-package-name
                          :one-mesh t
                          :prefix-name prefix-name
                          :parent-frame 
                          (gen-rvizable-name
                           (gen-rvizable-name "jsk_" 
                                              (string (send mapmodel :name)))
                           (concatenate string "/jsk_" 
                                        (string (send x :name))))))
            (eusmap-convert-target-models mapmodel))
    (write-map-rviz-config-file mapmodel (namestring (truename base-dir))
                                base-dir-package-name prefix-name)
    (when launch
      (write-room-object-tf-launch mapmodel base-dir
                                   :offset offset)
      (write-room-model-frame-dictionary-launch mapmodel base-dir
                                                base-dir-package-name
                                                prefix-name))
    (write-room-model-frame-dictionary-yaml mapmodel base-dir
                                            base-dir-package-name
                                            prefix-name)
    (when launch
      (write-map-launch-file mapmodel (namestring (truename base-dir))
                             base-dir-package-name prefix-name))
    ))

(defun eusmap-convert-target-models (mapmodel)
  (remove-duplicates
   (remove-if
    #'(lambda (x) (null (send x :name)))
    (remove-if
     #'(lambda (x) (derivedp x ground))
     (send mapmodel :all-objects)))
   :key #'(lambda (x) (send x :name)) :test #'equal))

(defun write-map-launch-file (mapmodel base-dir base-dir-package-name
                                       prefix-name)
  (let ((fname (format nil "~A/launch/~A.launch" 
                       base-dir
                       (send mapmodel :name)))
        (all-objects (eusmap-convert-target-models mapmodel)))
    (with-open-file (f fname :direction :output)
      (format f "<launch>~%")
      (format f "  <include file=\"$(find ~A)/launch/~A_tf.launch\" />~%"
              base-dir-package-name (send mapmodel :name))
      (format f "  <include file=\"$(find ~A)/launch/~A_mfd.launch\" />~%"
              base-dir-package-name (send mapmodel :name))
      ;; (dolist (o all-objects)
      ;;   (format f "  <include file=\"$(find ~A)/launch/~A.launch\" />~%"
      ;;           base-dir-package-name 
      ;;           (gen-rvizable-name prefix-name (string (send o :name)))))
      (format f "</launch>~%"))))

(defun write-map-rviz-config-file (mapmodel base-dir base-dir-package-name 
                                            prefix-name)
  (let ((fname (format nil "~A/rviz/~A.vcg" 
                       (namestring (truename base-dir))
                       (send mapmodel :name)))
        (all-objects (remove-duplicates
                      (remove-if #'null
                                 (send-all 
                                  (eusmap-convert-target-models mapmodel)
                                  :name))
                      :test #'equal)))
    (with-open-file (f fname :direction :output)
      (format f "Property\ Grid\ State=expanded=.Global Options")
      (format f "~%")
      (dotimes (i (length all-objects))
        (let ((prefix-string 
               (gen-rvizable-name "" (string (elt all-objects i)))))
          (format f "~A.Alpha=1~%" prefix-string)
          (format f "~A.Collision\\ Enabled=0~%" prefix-string)
          (format f "~A.Enabled=1~%" prefix-string)
          (format f "~A.Update\\ Rate=10~%" prefix-string)
          (format f "~A.Visual\\ Enabled=1~%" prefix-string)
          (format f "~A.Robot\\ Description=~A~%" prefix-string
                  (gen-rvizable-name prefix-name
                                     (string (elt all-objects i))))))
      (dotimes (i (length all-objects))
        (format f "[Display~A]~%" i)
        (format f "Name=~A~%"
                (gen-rvizable-name "" (string (elt all-objects i))))
        (format f "Package=rviz~%")
        (format f "ClassName=rviz::RobotModelDisplay~%")
        )
      )
    fname))

(defun gen-rvizable-name (prefix str)
  (coerce (mapcar #'(lambda (x) (if (= x #\-) #\_ x))
                  (coerce (concatenate string prefix str) cons))
          string))

;; (load "eusurdf.l")
;; (setq *pr2* (car (urdf2eus "full_pr2.urdf")))
;; (setq *nao* (car (urdf2eus "~/ros/alufr-ros-pkg-read-only/nao/nao_description/urdf/nao_robot.xml")))
;; (progn (hrp2jsk-simple) (eus2urdf *hrp2* "~/prog/hrp2/ros/hrp2_common/hrp2_description" "hrp2_description"))
;; (setq *hrp2* (car (urdf2eus "~/prog/hrp2/ros/hrp2_common/hrp2_description/urdf/HRP2JSK.urdf")))
;; (eusmap2urdf (make-room602-scene) "~/ros/boxturtle/jsk-ros-pkg/jsk_object_models/jsk_room602" "jsk_room602" :offset (make-coords :pos #f(-7000 -3500 0)))
