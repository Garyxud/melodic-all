(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "models/foldable-desk-object.l")
(load "models/petbottle-object.l")

(defun init-collision-demo (&optional (robot *sr*) (link-name "WAIST_LINK0"))
  (setq *co* (instance collision-object-publisher :init))

  (setq *table* (instance foldable-desk-object :init))
  (setq *bottle* (instance petbottle-object :init))
  (send *bottle* :translate #f(-100.0 140.0 705.0))

  (setq *robot* robot)
  (setq *link-name* link-name)
  (objects (list *table* *bottle*))
  )

(defun pub-objects ()
  (pub-obj *table*)
  (pub-obj *bottle*)
  )

(defun clear-objects ()
  (send *co* :clear-all)
  )

(defun pub-obj (obj &optional (robot *robot*) (link-name *link-name*)) ;; need link
  (let ((cds (send (send robot
                         (intern (format nil "~A_LK" (string-upcase link-name))
                                 *keyword-package*) :worldcoords)
                   :transformation (send obj :worldcoords))))
    (send *co* :add-object obj :frame_id link-name :relative-pose cds)
    obj))

(warn "
(init-collision-demo *your-robot* \"ROOT_LINKNAME\")
(pub-objects)
;; (clear-objects)
")
