;; speak-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(ros::roseus "test_speak")

(require :robot-interface "package://pr2eus/robot-interface.l")
(load "irteus/demo/sample-robot-model.l")

(init-unit-test)

(defclass sample-robot-interface
  :super robot-interface
  :slots ())
(defmethod sample-robot-interface
  (:init
   (&rest args)
   (send-super* :init :robot sample-robot args)
   self))

(unless (boundp '*ri*)
  (setq *ri* (instance sample-robot-interface :init)))

(deftest test-speak-en ()
  (assert (speak-en "hello, world"))
  (assert (speak-en "hello, world" :wait t))
)

(deftest test-speak-jp ()
  (assert (speak-jp "こんにちは"))
  (assert (speak-jp "こんにちは" :wait t))
)

(deftest test-ri-speak-en ()
  (assert (send *ri* :speak-en "hello, world"))
  (assert (send *ri* :speak-en "hello, world" :wait t)))
  )

(deftest test-ri-speak-jp ()
  (assert (send *ri* :speak-jp "やあ"))
  (assert (send *ri* :speak-jp "やあ" :wait t))
)

(deftest test-speak-file ()
  (assert (speak (pathname "/usr/share/sounds/alsa/Front_Left.wav"))))

(deftest test-ri-speak-file ()
  (assert (send *ri* :speak (pathname "/usr/share/sounds/alsa/Front_Left.wav"))))

(deftest test-speak-number ()
  (assert (speak 1)))

(deftest test-ri-speak-number ()
  (assert (send *ri* :speak 1)))

(run-all-tests)
(ros::sleep 2) ;; wait for last message actually send out
(exit)

