;; conversion-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defvar *eusurdf-package-dir* (unix:getwd))

(defun string-split (str &optional (delim #\Space))
  (let (ret sub c)
    (with-input-from-string (s str)
      (while t
       (cond
         ((null (setq c (read-char s nil)))
          (unless (null sub)
            (push (coerce (reverse sub) string) ret))
          (return-from string-split (reverse ret)))
         ((eq c delim)
          (push (coerce (reverse sub) string) ret)
          (setq sub nil))
         (t (push c sub)))))))

(defun string-join (jstr strlst)
  (if (<= (length strlst) 0) (return-from string-join nil))
  (let ((retstr (string (car strlst))))
    (dolist (s (cdr strlst))
      (setq retstr (concatenate string retstr jstr (string s))))
    retstr))

(defmethod pathname
  (:parent-directory ()
    (make-pathname :directory (butlast (send self :directory)))))

(defun errorf (fmt &rest args)
  (apply #'format *error-output* fmt args)
  (exit 1))

(defun make-dirs (dir)
  (let ((p (pathname dir)))
    (case (probe-file p)
      (:directory t)
      (:file (errorf "could not make directory at ~A. file already exists" p))
      (nil
       (make-dirs (send p :parent-directory))
       (unix:mkdir (send p :namestring)))
      (t (errorf "could not make directory ~A. unexpected error" p)))))

(defun make-indent-string (level)
  (with-output-to-string (s)
    (dotimes (i level)
      (format s "  "))
    (get-output-stream-string s)))

(defparameter *textured-models* nil)
(defun use-textured-model-p (model-name)
  (unless *textured-models*
    (unless (probe-file (format nil "~A/textured_models" *eusurdf-package-dir*))
      (return-from use-textured-model-p nil))
    (setq *textured-models* (directory (format nil "~A/textured_models" *eusurdf-package-dir*))))
  (cond
    ((find (concatenate string model-name "_fixed") *textured-models* :test #'string=)
     :fixed)
    ((find (concatenate string model-name "_static") *textured-models* :test #'string=)
     :static)
    ((find model-name *textured-models* :test #'string=)
     t)
    (t nil)))

(provide :conversion-utils) ;; end of conversion-utils.l
