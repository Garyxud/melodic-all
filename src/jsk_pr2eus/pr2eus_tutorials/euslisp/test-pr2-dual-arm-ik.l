#!/usr/bin/env roseus
(load "package://pr2eus/pr2-interface.l")
(pr2)
(objects *pr2*)


(setq c1 (send *pr2* :larm :end-coords :copy-worldcoords))
(send c1 :translate #f(500 200 0) :world)
(setq c2 (send *pr2* :rarm :end-coords :copy-worldcoords))
(send c2 :translate #f(500 0 -50) :world)

(send *pr2* :inverse-kinematics (list  c1
                                      c2)
      :move-target (list  (send *pr2* :larm :end-coords)
                         (send *pr2* :rarm :end-coords))
      :link-list (list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                       (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
      :rotation-axis (list nil nil)
      :base-range (list :min (float-vector -10000 -10000 -80)
                        :max (float-vector 10000 10000  80))
      :use-base t
      :debug-view :no-message)
