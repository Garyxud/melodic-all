euscollada ROS Launch Files
===========================

**Description:** collada_eus

  
  
       euscollada
  
    

**License:** BSD

euscollada-test.launch
----------------------

.. code-block:: bash

  roslaunch euscollada euscollada-test.launch


Convert collada robot to euslisp robot model

PR2 example

.. code-block:: bash

  rosrun euscollada pr2.sh

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euscollada/html/_images/pr2_test
  :width: 400

.. code-block:: bash

  rosrun euscollada collada2eus <collada model file name> <euslisp model file name>

Mitsubishi PA10
https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/mitsubishi-pa10.zae

.. image:: test/mitsubishi-pa10.png
  :width: 400

Unimate PUMA Arm
https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/unimation-pumaarm.zae

.. image:: test/unimation-pumaarm.png
  :width: 400

Care-O-Bot
https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/care-o-bot3.zae

.. image:: test/care-o-bot3.png
  :width: 400

Darpa ARM
https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/darpa-arm.zae

.. image:: test/darpa-arm.png
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
    
    <test args="$(find euscollada)/test/euscollada-pr2-test.l" launch-prefix="glc-capture --start --out=$(find euscollada)/build/pr2_test.glc" pkg="roseus" test-name="euscollada_pr2_test" type="roseus" />
    <test args="$(find euscollada)/build/pr2_test.glc" pkg="jsk_tools" test-name="z_pr2_test" time-limit="1000" type="glc_encode.sh" />
  
  </launch>

