(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(defun generate-eusmodel-from-collada (col-file eus-file &key (yaml-file))
  (let ((strm (piped-fork
               (if yaml-file
                   (format nil "rosrun euscollada collada2eus ~A ~A ~A && echo finished"
                           col-file yaml-file eus-file)
                 (format nil "rosrun euscollada collada2eus ~A ~A && echo finished"
                         col-file eus-file)))))
    ;; (assert (select-stream (list strm) 240) (format nil ";; faild to generate ~A within 240 second" eus-file)) ;; why return nil immidiately
    (dotimes (i 240)
      (when
	  (while (and (select-stream (list strm) 1)
		      (not (eq :eof (read-line strm nil :eof))))
	    (return t))
	(return))
      (assert (/= i  239) (format nil ";; faild to generate ~A within 240 second" eus-file))
      (unix::sleep 1)
      )
    (unix::sleep 1)
    (close strm)
    ))

(deftest generate-pr2-model-test
  ;;
  (if (eq 'yes
          (let ((strm (piped-fork "if [ -e `rospack find pr2_mechanism_model`/pr2.urdf ]; then echo yes; else echo no; fi;")))
            (prog1 (read strm)
              (close strm))))
      (let ((strm (piped-fork "rosrun collada_urdf urdf_to_collada `rospack find pr2_mechanism_model`/pr2.urdf /tmp/pr2.dae")))
        (assert (select-stream (list strm) 120) ";; ;; faild to generate /tmp/pr2.dae within 120 second")
        (close strm))
    (let ((strm (piped-fork "rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro > /tmp/pr2.$$.urdf && rosrun collada_urdf urdf_to_collada /tmp/pr2.$$.urdf /tmp/pr2.dae")))
      (assert (select-stream (list strm) 120) ";; faild to generate /tmp/pr2.dae within 120 second")
      (close strm))
    )
  ;;
  (assert (probe-file "/tmp/pr2.dae") ";; faild to generate /tmp/pr2.dae, file does not exist")
  (generate-eusmodel-from-collada "/tmp/pr2.dae" "/tmp/pr2.l"
                                  :yaml-file "$(rospack find euscollada)/pr2.yaml")
  ;;
  (assert (probe-file "/tmp/pr2.l") ";; faild to generate /tmp/pr2.l, file does not exist")
  (load "/tmp/pr2.l")
  (pr2)
  )

(load "irteus/irtmodel.l")
(deftest pr2-eusmodel-validity-check
  (eusmodel-validity-check-one *pr2*))

(deftest generate-sample-robot-model-test
  (generate-eusmodel-from-collada
   "$(rospack find openhrp3)/share/OpenHRP-3.1/sample/model/sample1.dae"
   "/tmp/samplerobot.l" :yaml-file "$(rospack find euscollada)/test/samplerobot.yaml")

  (assert (probe-file "/tmp/samplerobot.l") ";; faild to generate /tmp/samplerobot.l, file does not exist")
  (load "/tmp/samplerobot.l")
  (samplerobot)
  (send *samplerobot* :init-pose)
  (eusmodel-validity-check-one *samplerobot*)
  (send *samplerobot* :fix-leg-to-coords (make-coords))
  (assert (eps= 1.308000e+05 (send *samplerobot* :weight))
          ";; sample robot has not valid weight ~A" (send *samplerobot* :weight))
  (assert (eps-v= #f(2.32416 0.0 791.942) (send *samplerobot* :centroid))
          ";; sample robot has not valid centroid ~A" (send *samplerobot* :centroid))
  )

(deftest generate-sample-robot-no-yaml-test
  (generate-eusmodel-from-collada
   "$(rospack find openhrp3)/share/OpenHRP-3.1/sample/model/sample1.dae"
   "/tmp/samplerobot_no_yaml.l")

  (assert (probe-file "/tmp/samplerobot_no_yaml.l") ";; faild to generate /tmp/samplerobot_no_yaml.l, file does not exist")
  (load "/tmp/samplerobot_no_yaml.l")
  (samplerobot)

  (eusmodel-validity-check-one *samplerobot*)
  (send *samplerobot* :init-pose)
  (send *samplerobot* :move-coords (make-coords) (send (car (send *samplerobot* :links)) :worldcoords))
  (assert (eps= 1.308000e+05 (send *samplerobot* :weight))
          ";; sample robot has not valid weight ~A" (send *samplerobot* :weight))
  (assert (eps-v= #f(2.32416 0.0 68.4419) (send *samplerobot* :centroid))
          ";; sample robot has not valid centroid ~A" (send *samplerobot* :centroid))
  )

(run-all-tests)
(exit)
