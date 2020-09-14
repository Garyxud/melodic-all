(unless (find-package "SLIME") (make-package "SLIME"))
(in-package "SLIME")

;; Generate  *help-hash*
;; (setq help::*eus-tex-dir* "/path/to/latex/")
(help '+ nil nil)

;; UTILITY FUNCTIONS
(defun position-from-end (item seq &rest key-args)
  (aand
   (apply #'position item (reverse seq) key-args)
   (- (length seq) it 1)))

(defun read-copy (obj name)
  (cond
    ((stringp obj)
     (if (and (> (length obj) 0) (eq (elt obj 0) #\())
         (cons name (read-from-string obj))))
    ((consp obj)
     (cons name (copy-list obj)))))

(defun maybe-downcase (str)
  (if (eql *print-case* :downcase)
      (string-downcase str)
      str))

;; Anaphoric macros
(defmacro aif (test-form then-form &optional else-form)
  `(let ((it ,test-form))
     (if it ,then-form ,else-form)))

(defmacro aand (&rest args)
  (cond ((null args) t)
	((null (cdr args)) (car args))
	(t `(aif ,(car args) (aand ,@(cdr args))))))

;; Symbol related
(defun split-package (name)
  ;; TODO: read |escaped\|forms|
  (let* ((pos (position-from-end #\: name))
         (internal (and pos (plusp pos) (= #\: (elt name (1- pos))))))
    (case pos
      (nil (values nil name))
      (0 (values "KEYWORD" (subseq name (1+ pos))))
      (t (values (string-upcase (subseq name 0 (if internal (1- pos) pos)))
                 (subseq name (1+ pos)))))))

(defmacro slime-do-symbols (var &rest body)
  ;; var: (variable-name &optional package result)
  `(if ,(cadr var)
       (if (find-package ,(cadr var)) (do-symbols ,var ,@body))
       (do-callable-symbols (,(car var) ,(caddr var)) ,@body)))

(defmacro do-callable-symbols (var &rest forms)
  ;; var: (variable-name &optional result)
  (let ((apackage (gensym "DOALLSYM")))
    `(progn
       (do-symbols (,(car var) *package*) ,@forms)
       (dolist (,apackage (*package* . use) ,(cadr var))
	 (do-external-symbols (,(car var) ,apackage) ,@forms)))))

(defun find-external-symbol (name package)
  (do-external-symbols (sym package)
    (if (string= (symbol-pname sym) name)
        (return-from find-external-symbol sym))))

(defun find-callable-symbol (name)
  (multiple-value-bind (pack name) (split-package name)
    (setq name (string-upcase name))
    (if pack
        (and (find-package pack) (find-symbol name pack))
        (or (find-symbol name) 
            (let ((packs (package-use *package*)))
              (dolist (p packs)
                (aif (find-external-symbol name p)
                     (return-from find-callable-symbol it))))))))

(defmacro with-callable-symbol-value (var-lst &rest body)
  ;; var-lst is in form (val-var symbol-name &optional result) OR
  ;; ((sym-var val-var) symbol-name &optional result)
  (let ((val-var (if (consp (car var-lst))
                     (cadar var-lst)
                     (car var-lst)))
        (sym-var (if (consp (car var-lst))
                     (caar var-lst)
                     (gensym))))
    `(let ((,sym-var (find-callable-symbol ,(cadr var-lst))))
       (if (and ,sym-var (boundp ,sym-var))
           (let ((,val-var (if (< (symbol-vtype ,sym-var) 3)
                               (symbol-value ,sym-var)
                               (elt (system::thread-specials) (symbol-vtype ,sym-var)))))
             ,@body
             ,@(cddr var-lst))))))

;; Prefix related
(defun append-common (match-lst start-len)
  (cond
    ((cdr match-lst)
     (list match-lst (subseq (car match-lst) 0 (get-common match-lst start-len))))
    (match-lst ;; one element list
     (list match-lst (car match-lst)))
    (t nil)))

(defun get-common (lst &optional (start 0))
  (do* ((min (reduce #'min (mapcar #'length lst)))
	(i start (1+ i))
	(c nil))
       ((>= i min) i)
    (setq c (elt (car lst) i))
    (dolist (s lst)
      (unless (eq (elt s i) c) (return-from get-common i)))))

(defun find-prefix (prefix lst &optional (start 0))
  (let ((len (length prefix)))
    (append-common
     (remove-if-not
      #'(lambda (str) (string-equal prefix str :start1 start :end1 len :start2 start :end2 len))
      lst)
     len)))

;; Help search
(defun get-help (name &optional class)
  (flet ((read-item (item)
           (case (help::help-item-type item)
             ((1 2 3 6) ;; method, function, macro or special form
              (read-from-string (format nil "(~a ~a)" name (car (send item :read-help))))))))
    (let ((name (string-downcase name))
          (class (and class (string-downcase (send class :name)))))
      (if class
          (aand (gethash class help::*help-hash*)
                (gethash name (help::help-item-mhash it))
                (read-item it))
          (aand (gethash name help::*help-hash*)
                (read-item it))))))

;; Properties list
(defun lambda-list (name)
  (or (get-help name)
      (let ((sym (find-callable-symbol name)))
        (if sym
            (or (get-help (format nil "~a" sym))
                (if (and (fboundp sym) (consp (symbol-function sym)))
                    (cons sym (copy-list (cadr (symbol-function sym)))))
                (aif (assoc :function-documentation (send sym :plist))
                     (read-copy (cdr it) sym)))))))

(defun method-lambda-list (obj name)
  (let* ((sym (or (find-symbol (string-upcase (subseq name 1)) *keyword-package*)
                  (return-from method-lambda-list nil)))
         (method (find-method obj sym))
         (class (car method))
         (def (second (second method)))
         (arglist (or
                   (get-help name class)
                   (if (listp def) (cons sym (copy-list def)))
                   (aif (assoc class (get sym :method-documentation))
                        (read-copy (cdr it) sym)))))
    (if (and (eq (second arglist) 'self) (eq (third arglist) 'class))
        (cons (car arglist) (cdddr arglist))
        arglist)))

(defun keyword-list (arglist &optional start)
  (let ((pos (or start (aand (position '&key arglist) (1+ it)))))
    (if pos
        (mapcar #'(lambda (val)
                    (let ((key (if (consp val) (car val) val)))
                      (if (consp key) ;; &key ((:keyword kwd) 10)
                          (format nil "~a" (car key))
                          (format nil ":~a" key))))
                (nthcdr pos arglist)))))

(defun method-list (obj)
  (let* ((class (if (classp obj) obj (class obj)))
         (meths (mapcar #'car (send class :all-methods))))
    (mapcar #'(lambda (val) (format nil "~a" val))
            (remove-duplicates meths))))


;; CONNECTION-INFO
(defun implementation-version ()
  ;; Remove color from the console,
  ;; usually trailing from roseus init message
  (format t "~c[0m" #\Esc)
  (lisp-implementation-version))


;; REPL-PROMPT
(defvar last-prompt)
(defun slime-prompt ()
  (let ((prompt
         (string-right-trim (list #\space #\$ #\:)
          (with-output-to-string (s) (lisp::prompt s)))))
    (unless (equal prompt last-prompt)
      (setq last-prompt prompt)
      (list (package-name *package*) prompt))))


;; SWANK-SIMPLE-COMPLETIONS
(defun slime-find-symbol (start &optional prefix)
  (multiple-value-bind (pack str) (split-package start)
    (let ((len (length str))
          str-lst)
      (unless (or pack prefix)
        (mapc #'(lambda (p) (let ((name (package-name p)))
                              (if (and (string-equal str name :end2 len)
                                       (> (p . intsymcount) 0))
                                  (push (format nil "~a:" (maybe-downcase name))
                                        str-lst))))
              (list-all-packages)))
      (slime-do-symbols (sym pack)
                        (let ((sym-str (maybe-downcase (send sym :pname))))
                          (when (string-equal str sym-str :start1 0 :end1 len :start2 0 :end2 len)
                            (push (concatenate string prefix start (subseq sym-str len)) str-lst))))
      (append-common (sort str-lst #'string<) len))))


;; SWANK-AUTODOC
(defun autodoc (name &optional cursor form)
  (let ((last (car (last form))))
    (cond
      ((not (stringp last))
       (function-autodoc name cursor form))
      ((string= last "")
       (function-autodoc name (1- cursor) (butlast form)))
      ((and (> cursor 1)
            (with-callable-symbol-value ((sym value) last)
              (unless (eq (symbol-package sym) *keyword-package*)
                (format nil "~a => ~s" last value)))))
      (t
       (function-autodoc name (1- cursor) form)))))

(defun function-autodoc (name &optional cursor form)
  (let* ((arglist
          (aif (method-lambda-list-from-form form)
               (prog1 it
                 (if cursor (decf cursor 2)))
               (lambda-list name)))
         (len (length arglist)))
    (when arglist
      (unless cursor (return-from function-autodoc arglist))
      (dotimes (i (min len (1+ cursor)))
        (case (nth i arglist)
          (&rest
           (aif (position '&key arglist :start (1+ i))
                (return-from function-autodoc (set-keyword arglist it (car (last form))))
                (return-from function-autodoc (set-cursor arglist (1+ i)))))
          (&optional
           (incf cursor))
          (&key
           (return-from function-autodoc (set-keyword arglist i (car (last form)))))))
      (if (or (zerop cursor) (>= cursor len))
          arglist
          (set-cursor arglist cursor)))))

(defun method-lambda-list-from-form (form)
  ;; form: '("send" "obj" ":selector" ...)
  (if (and (cddr form)
           (string-equal (first form) "send")
           (stringp (second form))
           (stringp (third form)))
      (with-callable-symbol-value (value (cadr form))
        (method-lambda-list value (third form)))))

(defun set-cursor (lst pos)
  ;; TODO: use string not symbols
  (list-insert (intern "<===" *package*) (1+ pos) lst)
  (list-insert (intern "===>" *package*) pos lst))

(defun set-keyword (arglist start item)
  (let* ((keys (keyword-list arglist start))
         (pos (and (stringp item)
                   (position item keys :test #'string-equal))))
    (if pos
        (set-cursor arglist (+ start pos))
        arglist)))


;; SWANK-COMPLETIONS-FOR-KEYWORD
(defun slime-find-keyword (start &optional form)
  (cond
    ((null form)
     (let ((*package* *keyword-package*))
       (slime-find-symbol (subseq start 1) ":")))
    ((string-equal (car form) "send")
     (if (stringp (cadr form))
         (with-callable-symbol-value (value (cadr form))
           (if (cddr form)
               (let ((keys (keyword-list (method-lambda-list-from-form form))))
                 (find-prefix start keys 1))
               (find-prefix start (method-list value) 1)))))
    (t
     (let ((keys (keyword-list (lambda-list (car form)))))
       (find-prefix start keys 1)))))


;; SWANK-COMPLETIONS-FOR-CHARACTER
(defun slime-find-character (start)
  (find-prefix start *char-lst*))

(defconstant *char-lst*
  '("Space" "Newline" "Linefeed"
    "Backspace" "Delete" "Rubout"
    "Return" "Page" "Formfeed"
    "Esc" "Escape" "Tab"
    "Left-Paren" "Right-Paren" "Lparen" "Rparen"
    "Bell"  "Null" "SOH" "STX" "ETX"))


;; SWANK-DESCRIBE-SYMBOL
(defun slime-describe-symbol (name)
  (let* ((sym (find-callable-symbol name))
         (help-name (string-downcase (format nil "~a" sym))))
    (unless sym
      (error "symbol not found"))
    (with-output-to-string (s)
      (when (gethash help-name help::*help-hash*)
        (help help-name nil s)
        (terpri s))
      (format s "PROPERTIES~%~%")
      (describe sym s))))


;; SWANK-EXPAND-1
(defun slime-macroexpand (form)
  (with-output-to-string (s)
    (pprint (macroexpand form) s)))


;; SWANK-LIST-ALL-PACKAGE-NAMES
(defun slime-all-packages (&optional nicknames)
  (if nicknames
      (apply #'append (mapcar #'(lambda (p) (package-names p)) (list-all-packages)))
      (mapcar #'package-name (list-all-packages))))


;; SWANK-APROPOS-LIST
(defun slime-apropos-list (key &optional package)
  ;; (:designator "SB-UNICODE:CHAR-BLOCK" :function "Returns the Unicode block in which CHARACTER resides as a keyword.")
  ;; (:designator "SB-UNIX:EWOULDBLOCK" :variable :not-documented)
  (labels ((get-props (sym)
             (cond
               ((fboundp sym)
                (list
                 (cond
                   ((special-form-p sym) :special-operator)
                   ((macro-function sym) :macro)
                   (t :function))
                 (aif (get sym :function-documentation)
                      (if (stringp it)
                          it
                          (format nil "~s" it))
                      :not-documented)))
               ((and (boundp sym) (not (keywordp sym)))
                (list
                 (if (classp sym)
                     :class
                     :variable)
                 (aif (get sym :variable-documentation)
                      (if (stringp it)
                          it
                          (format nil "~s" it))
                      :not-documented)))))
           (list-props (sym)
             (aif (get-props sym)
                  (list (list*
                         :designator
                         (string-upcase (format nil "~a" sym))
                         it)))))
    (sort
     (mapcan #'list-props (apropos-list key package))
     #'string< #'cadr)))


;; SWANK-SET-PACKAGE
(defun set-package (name)
  (when (find-package name)
    (in-package name)
    (slime-prompt)))
