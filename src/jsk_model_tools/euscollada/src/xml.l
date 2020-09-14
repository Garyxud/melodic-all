;; xml utilities
;; implementation reference is: 
;; http://homepage1.nifty.com/blankspace/scheme/nsx.html
;; really thanks to nnakamura!

(defvar *collada-print-customize-symbols* nil)

(defun symbol->string (sym)
  (if (find sym *collada-print-customize-symbols*)
      (string sym)
    (string-downcase (string sym))))

(defun ->string (val)
  (format nil "~s" val))

(defun string-append (&rest args)
  (apply #'concatenate string args))

(defun make-attr (l ac)
  (if (null l)
      ac
    (make-attr
     (cdr l) (string-append ac " " (symbol->string (caar l))
                            "=\"" (if (stringp (cadar l))
                                      (cadar l)
                                    (->string (cadar l)))
                            "\""))))

(defun make-xml (x bef aft)
  (cond ((stringp x) (append bef (list x) aft))
        ((and (listp x) (symbolp (car x)) (not (null x)))
         (let ((tag (symbol->string (car x))))
           (if (and (not   (null (cdr x)))
                    (listp (cadr x))
                    (eq (caadr  x) '@))
               (make-xml (cddr x)
                         (append bef (list 'BO tag (make-attr (cdadr x) "") 'BC))
                         (append     (list 'EO tag 'EC) aft))
             (make-xml (cdr x)
                       (append bef (list 'BO tag 'BC))
                       (append     (list 'EO tag 'EC) aft)))))
        ((and (not (null x))
              (listp x)) (make-xml ()
                             (make-xml (car x) bef ())
                             (make-xml (cdr x) () aft)))
        (t (append bef aft))))

(defun sxml->xml (sxml)
  ;; main
  (make-xml sxml () ()))

;; utility
(defun xml-output-to-string-stream (ss l)
  (dolist (ll l)
    (if (stringp ll)
        (format ss ll)
      (format ss
              (case ll
                (BO    "<")
                (EO    "</")
                ((BC EC) ">~%"))))))

(defun cat-normal (l s)
  (cond ((null l) s)
        ((stringp (car l))
         (cat-normal (cdr l) (string-append s (car l))))
        (t (cat-normal (cdr l)
                          (string-append s (case (car l)
                                             (BO    "<")
                                             (EO    "</")
                                             ((BC EC) ">~%")))))))
;; utility
(defun cat-clark (l s i)
  (cond ((null l) s)
        ((stringp (car l))
         (cat-clark (cdr l) (string-append s (car l)) i))
        (else
         (case (car l)
           (BO    (cat-clark (cdr l) (string-append s "<" ) (+ i 1)))
           (EO    (cat-clark (cdr l) (string-append s "</") (- i 1)))
           ((BC EC) (cat-clark (cdr l)
                               (string-append s "~%" (make-string i #\ ) ">") i))))))

;; TODO:: Is there better implementations??
(defun sxml-child-nodes (sxml-node)
  (when (consp (cadr sxml-node))
    (let ((attr (assoc '@ sxml-node)))
      (cond
       (attr
        (setq sxml-node (remove attr sxml-node))
        (when (consp (cadr sxml-node))
          (cdr sxml-node)))
       (t
        (cdr sxml-node))))))

(defun sxml-all-children (sxml-node)
  (let ((cn (sxml-child-nodes sxml-node)))
    (cond
     (cn
      (append (list sxml-node) (mapcan #'(lambda (x) (when x (sxml-all-children x))) cn)))
     (t (list sxml-node)))))

(defun sxml-find-element-by-tagname (tag sxml-node)
  (remove-if-not #'(lambda (elem) (eq tag (car elem)))
                 (sxml-all-children sxml-node)))

(provide :xml)
