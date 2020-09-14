;;
;; eus2collada.l
;;
;;   output eus body object to collada (.dae)
;;

;; utility functions
(defun str+ (&rest args)
  (apply #'concatenate string args))

(defun addrstr (obj)
  (let ((addr (sys:address obj)))
    (if (eq (class addr) bignum)
	(format nil "~A" addr)
      (string addr))))

;; generate xml string from xml list

;; xml  := (tag attr node*)
;; tag  := atom
;; attr := (cons*) | nil
;; node := xml | atom | nil
(defun simple-list2xml (obj)
  (cond
   ((null obj) "")
   ((stringp obj) obj)
   ((vectorp obj)
    (apply #'concatenate string
	   (flatten (coerce (map vector #'(lambda(o)(list (string o) " ")) obj) cons))))
   ((atom obj) (string obj))
   ((listp obj)
    (let ((tag (car obj)) tagstr
	  (attrs (cadr obj)) attrstr
	  (nodes (cddr obj)) nodestr)
      (cond
       ((and (atom tag) (not (null tag)))
	(setq tagstr (string tag))
	(setq attrstr
	      (mapcar #'(lambda(a)
			  (format nil " ~A=\"~A\"" (string (car a))
				  (string (if (atom (cdr a)) (cdr a) (cadr a)))))
		      attrs))
	(setq nodestr (mapcar #'simple-list2xml nodes))
	(setq attrstr (apply #'concatenate string attrstr))
	(setq nodestr (apply #'concatenate string nodestr))
	(if nodes
	    (str+ "<" tagstr attrstr ">" nodestr "</" tagstr ">~%")
	  (str+ "<" tagstr attrstr "/>~%")))
       (t "")
       ))))
  )


;; from jskgl.l for cygwin
(defun extract-texture-and-coords (abody face0)
  (let ((lcoords (send abody :copy-coords))
        (c-tbl (get abody :gl-texturecoordinate2))
        (f-tbl (get abody :gl-textureimage2))
        tex-c tex-cs-y tex-cs-x img0)
    (cond
     ((setq img0 (gethash face0 f-tbl))
      (setq tex-c nil tex-cs-x nil tex-cs-y nil)
      ;;
      (dolist (el (send face0 :edges))
        (setq tex-c (gethash (list el face0) c-tbl))
        (push (aref tex-c 0) tex-cs-x)
        (push (aref tex-c 1) tex-cs-y))
      ;;
      (send abody :newcoords lcoords)
      (send abody :worldcoords)
      (list img0 (list (reverse tex-cs-x)(reverse tex-cs-y))))
     (t nil))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; library_images
;; save images as "dir/*.jpg"
(defun collada-image (img dir)
  (let* ((iid (addrstr img))
	(filename (str+ dir "/" iid ".jpg")))
    (send img :write-to-jpg-file filename)
    (list "image" (list (cons "id" (str+ "image-" iid))
			(cons "name" (str+ "image-" iid)))
      (list "init_from" nil
	(list "ref" nil (str+ iid ".jpg"))))
    ))

(defun body-textures (bd)
  (let ((hlst (get bd :gl-textureimage2)))
    (if hlst (send hlst :list)
      )))

;; library_effects
(defun collada-effect (col &optional img)
 (if (not col) (setq col (gl::find-color :white)))
 (let* ((id (addrstr (or img col)))
       (surface-id (str+ "surface-" id))
       (sampler-id (str+ "sampler-" id)))
  (list "effect" (list (cons "id" (str+ "effect-" id)))
   (list "profile_COMMON" nil
    (if img
	(list "newparam" (list (cons "sid" surface-id))
	 (list "surface" (list (cons "type" "2D"))
	   (list "init_from" nil (str+ "image-" id))
	   )))
    (if img
	(list "newparam" (list (cons "sid" sampler-id))
	 (list "sampler2D" nil
	   (list "source" nil surface-id)
	   )))
    (list "technique" (list (cons "sid" "blender"))
     (list "lambert" nil
      (list "emission" nil
       (list "color" nil (send col :emission) "1"))
      (list "ambient" nil
       (list "color" nil (send col :ambient)))
      (list "diffuse" nil
       (if img
	   (list "texture" (list (cons "texcoord" (str+ "TEX-" id))
				 (cons "texture" (str+ "sampler-" id))))
	 (list "color" nil (send col :diffuse))))
      (list "specular" nil
       (list "color" nil (send col :specular) "1"))
      (list "shininess" nil
       (list "float" nil (send col :shininess)))
      (list "reflectivity" nil
       (list "float" nil 1.0))
      (list "transparent" nil
       (list "color" nil #f(1 1 1 1)))
      (list "transparency" nil
       (list "float" nil (send col :transparency))))))))
 )

;; library_materials
(defun collada-material (col &optional img)
 (setq col (gl::find-color (or col :white)))
 (let ((id (addrstr (or img col))))
   (list "material" (list (cons "id" (str+ "material-" id)))
     (list "instance_effect" (list (cons "url" (str+ "#effect-" id)))))
   ))


;; primitives are <p> and <ph>
(defun collada-polygons (primitives msymbol vid nid
				    &key (texture nil))
  (append
   (list "polygons" (list (cons "count" (length primitives))
			  (cons "material" msymbol))
	 (list "input" (list (cons "semantic" "VERTEX")
			     (cons "source" (str+ "#" vid))
			     (cons "offset" 0)))
	 (list "input" (list (cons "semantic" "NORMAL")
			     (cons "source" (str+ "#" nid))
			     (cons "offset" 1)))
	 (if texture
	     (list "input" (list (cons "semantic" "TEXCOORD")
				 (cons "source" texture)
				 (cons "offset" 2)))))
   primitives))


(defun collada-source (vec id &rest args)
  (let ((count (length vec)) (stride (length args)))
   (list "source" (list (cons "id" id))
    (list "float_array" (list (cons "id" (str+ id "-array"))
			      (cons "count" count))
	  vec)
    (list "technique_common" nil
     (append
      (list "accessor" (list (cons "source" (str+ "#" id "-array"))
			     (cons "count" (/ count stride))
			     (cons "stride" stride)))
      (mapcar #'(lambda(x)
		 (list "param" (list (cons "name" x)
				     (cons "type" "float"))))
	      args))
))))


;; eus-body -> collada-mesh
(defun collada-mesh (obj)
  (let (objid pid nid vid textures
	      textured-faces faces polys
	      vertices normals result)
   (setq objid (addrstr obj))
   (setq pid (str+ "obj-" objid "-Pos"))
   (setq nid (str+ "obj-" objid "-Normal"))
   (setq vid (str+ "obj-" objid "-Vtx"))
   (setq textures (body-textures obj))
   (setq textured-faces (mapcar #'car textures))
   (setq faces (send obj :faces))
;   (setq polys (mapcan #'(lambda(f)(append (list f) (send f :holes))) faces))
   (setq polys faces)
   (setq vertices
	 (mapcar #'(lambda(f)(cdr (send f :vertices))) polys))
   (setq normals
	 (mapcar #'(lambda(f)(send f :normal)) polys))

   ;; POSITION
   (let* ((posvec (apply #'concatenate float-vector (flatten vertices))))
    (push ;; push to result
     (collada-source (scale 0.001 posvec) pid "X" "Y" "Z")
     result
     ))

   ;; NORMAL
   (let* ((norvec (apply #'concatenate float-vector normals)))
     (push ;; push to result
      (collada-source norvec nid "X" "Y" "Z")
      result
      ))

   ;; UV list
   (dolist (fs textured-faces)
    (let* ((tex-pos (extract-texture-and-coords obj fs))
	   (iid (str+ "texture-" (addrstr (car tex-pos)) "-UV"))
	   (pos-x (caadr tex-pos))
	   (pos-y (mapcar #'(lambda(p)(- 1 p)) (car (cdadr tex-pos)))) ;; for flip in y axis
	   (uvvec (concatenate float-vector (mapcan #'list pos-x pos-y))))
      (push ;; push to result
       (collada-source uvvec iid "S" "T")
       result
       )))

   ;; VERICES
   (push ;; push to result
     (list "vertices" (list (cons "id" vid))
       (list "input" (list (cons "semantic" "POSITION")
			   (cons "source" (str+ "#" pid)))))
      result)

    ;; POLYGONS (holes not yet)
    (let (plst tex (vcnt 0) iid)
      (dotimes (i (length polys))
	(setq tex (if (assoc (elt polys i) textures) t nil))
	(if tex (setq iid (addrstr (cdr (assoc (elt polys i) textures)))))
	(let* ((ver (elt vertices i))
	       (num (length ver))
	       (stride (if tex 3 2))
	       (iv (instantiate integer-vector (* stride num))))
	  (dotimes (j num)
	    (setf (elt iv (1+ (* stride j))) i)
	    (setf (elt iv (* stride j)) vcnt)
	    (inc vcnt))
	  (when tex
	    (dotimes (j num)
	      (setf (elt iv (+ 2 (* stride j))) j)))
;	  (if (derivedp (elt polys i) hole)
	  (if tex
	      (push ;; push to result
	       (collada-polygons
		  (list (list "p" nil iv))
		  (str+ "texturesymbol-" iid) vid nid
		  :texture (str+ "#texture-" iid "-UV"))
	       result)
	    (push (list "p" nil iv) plst))
	  ))

      (push ;; push to result
       (collada-polygons
	  plst
	  (str+ "bodysymbol-" objid) vid nid)
       result)
      )

    ;; output
    (list "geometry" (list (cons "id" (str+ "obj-" objid))
			   (cons "name" (str+ "obj-" objid)))
	  (append (list "mesh" nil) (reverse result)))
    ))


(defun collada-geoinstance (bd)
  (let* ((bid (addrstr bd)) bind-materials
	 (col (gl::find-color (or (get bd :face-color) :white)))
	 (imgs (append '(nil) (mapcar #'cdr (body-textures bd)))))
    (setq bind-materials
     (mapcar
       #'(lambda(img)
	(let ((symb (if img (str+ "texturesymbol-" (addrstr img))
		      (str+ "bodysymbol-" bid))))
	  (list "bind_material" nil
	   (list "technique_common" nil
	    (list "instance_material" (list (cons "symbol" symb)
					    (cons "target" (str+ "#material-" (addrstr (or img col))))))))))
     imgs))

    (append
     (list "instance_geometry" (list (cons "url" (str+ "#obj-" bid))))
     bind-materials)
    ))


;; visual_scene
(defun collada-node (bd)
  (let ((bid (addrstr bd))
	(rot (send bd :worldrot)))
    (list "node" (list (cons "id" (str+ "node-" bid))
		       (cons "name" (str+ "node-" bid)))
     (list "translate" nil #f(0 0 0))
     (list "rotate" nil #f(0 0 1 0))
     (list "rotate" nil #f(0 1 0 0))
     (list "rotate" nil #f(1 0 0 0))
     '("scale" nil #i(1 1 1))
     (collada-geoinstance bd))
    ))


(defun simple-collada (obj dir)
  (let (bds faces images material-col material-img materials)
    (setq bds (flatten
		 (send-all (send obj :bodies) :bodies)))
    (dolist (b bds)
      (let ((c (gl::find-color (or (get b :face-color) :white)))
	    (imgs (mapcar #'cdr (body-textures b)))
	    (ret nil))
       (if (= 0 (count (sys:address c) materials :key #'car))
	   (push (list (sys:address c) c nil) materials))
       (dolist (img imgs)
	 (if (= 0 (count (sys:address img) materials :key #'car))
	     (push (list (sys:address img) c img) materials)))))
    (setq material-col (mapcar #'cadr materials))
    (setq material-img (mapcar #'caddr materials))
    (setq images (remove nil (mapcar #'caddr materials)))

    ;; this is collada xml
 (list "COLLADA" '(("xmlns" "http://www.collada.org/2005/11/COLLADASchema")
		   ("version" "1.5.0"))
  (list "asset" nil
   (list "keywords" nil "euslisp model " (send (class obj) :name))
   (list "title" nil (send obj :name)))

  (if images
  (apply #'list "library_images" nil
   (mapcar #'(lambda(im)(collada-image im dir)) images)))
  (apply #'list "library_effects" nil
   (mapcar #'collada-effect material-col material-img))
  (apply #'list "library_materials" nil
   (mapcar #'collada-material material-col material-img))
  (apply #'list "library_geometries" nil
   (mapcar #'collada-mesh bds))
  (list "library_visual_scenes" nil
   (apply #'list "visual_scene" '(("id" "DefaultScene"))
    (mapcar #'collada-node bds)))
  '("scene" nil
    ("instance_visual_scene" (("url" "#DefaultScene"))
     )))
 ))

(defun eus2collada (obj dir)
  (let ((is-coords (derivedp obj coordinates)) prev-coords
	(has-links (derivedp obj articulated-link))
	ret)
    (when is-coords
      (setq prev-coords (send obj :copy-worldcoords))
      (send obj :move-to (make-coords) :world)
      (send obj :worldpos)
      (if has-links (send-all (send obj :links) :worldpos)))
    ;;
    (setq ret (simple-list2xml (simple-collada obj dir)))
    ;;
    (when is-coords
      (send obj :move-to prev-coords :world)
      (send obj :worldpos)
      (if has-links (send-all (send obj :links) :worldpos)))
    ret
    ))

;;
(defun write-collada-file (obj dir &key (name nil))
  (let* ((fname (or name (string (sxhash obj))))
	(filename (str+ dir "/" fname  ".dae")))
    (with-open-file (f filename :direction :output)
		    (format f (eus2collada obj dir)))
    filename
    ))


;; sample
;; (write-collada-file (make-cube 100 100 100) "/home/leus/work")
;; (write-collada-file (hrp2jsk-simple) "/home/leus/work")

;; TODO
;; holes in face
;; texture image embedding
