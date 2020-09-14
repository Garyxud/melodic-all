#!/usr/bin/env roseus
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "pr2_main")

(defmethod pr2-robot
  (:initial-pose ()
   (send self :angle-vector
         (float-vector 12.0 45.0 60.0 0.0 -120.0 0.0 -45.0 0.0 -45.0 60.0 0.0 -120.0 0.0 -45.0 0.0 0.0 35.0))
   ))

#|
;; make limb's index vector
(map integer-vector
     #'(lambda (j)
         (position j (send *pr2* :joint-list)))
     (send *pr2* limb :joint-list))
|#
(defmethod robot-interface
  (:error-coords (limb)
   (let ((pt (send self :state :potentio-vector))
         (ds (send self :state-vector :desired))
         (limb-pt (send robot limb :angle-vector))
         (pt-cds (send robot limb :end-coords))
         limb-ds ds-cds diff-cds)
     (send robot :angle-vector ds)
     (setq ds-cds (send robot limb :end-coords)
           limb-ds (send robot limb :angle-vector))
     (setq diff-cds (send pt-cds :transformation ds-cds))
     (setf (get diff-cds :diff-angle-vector) (v- limb-pt limb-ds))
     (setf (get diff-cds :limb) limb)
     (send robot :angle-vector pt)
     diff-cds
     ))
  )

;; initial pose
(defun goto-init-pose (&key (wait t) (tm 4000))
  (send *pr2* :initial-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) tm)
  (if wait (send *ri* :wait-interpolation)))

(warn "
(pr2-init)
(goto-init-pose)
")
