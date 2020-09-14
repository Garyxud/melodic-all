(load "unittest.l")
(init-unit-test)
(load "package://eusurdf/euslisp/convert-eus-to-urdf.l")

(deftest irteus2urdf-for-gazebo-test ()
  (load "models/laundry-machine-object.l")
  (irteus2urdf-for-gazebo (laundry-machine) :name "irteus2urdf_test")
  ;; check existence of urdf model directory
  (assert (equal (probe-file (ros::resolve-ros-path "package://eusurdf/models/irteus2urdf_test")) :directory))
  ;; check existence of urdf model file
  (assert (equal (probe-file (ros::resolve-ros-path "package://eusurdf/models/irteus2urdf_test/model.urdf")) :file))
  ;; check file size of urdf model file
  (> (file-size (ros::resolve-ros-path "package://eusurdf/models/irteus2urdf_test/model.urdf")) 0)
  )

(deftest irteus2urdf-room-for-gazebo-test ()
  (load "models/room610-scene.l")
  (irteus2urdf-room-for-gazebo (room610) (ros::resolve-ros-path "package://eusurdf/worlds/room610.world"))
  )

(deftest generate-room-models-test ()
  (generate-room-models "room610")
  ;; check existence of world file
  (assert (equal (probe-file (ros::resolve-ros-path "package://eusurdf/worlds/room610.world")) :file))
  )

(run-all-tests)
(exit)
