(defun pr2-setup
  ()
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  (setq *robot* *pr2*)
  (objects (list *robot*))
  )

(defun hrp2jsk-setup
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
  (hrp2jsk-init)
  (setq *robot* *hrp2jsk*)
  (objects (list *robot*))
  (send *ri* :start-auto-balancer)
  )
