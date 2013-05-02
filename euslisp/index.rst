euslisp ROS Launch Files
========================

**Description:** euslisp

  
  
       euslisp and irteus
  
    

**License:** BSD

test-euslisp.launch
-------------------

.. code-block:: bash

  roslaunch euslisp test-euslisp.launch


  

Contents
########

.. code-block:: xml

  <launch>
  
    <test args="$(find euslisp)/test/sort.l" pkg="euslisp" test-name="test_sortgc" type="irteusgl" />
    <test args="$(find euslisp)/test/bignum.l" pkg="euslisp" test-name="test_bignum" type="irteusgl" />
    <test args="$(find euslisp)/test/vector.l" pkg="euslisp" test-name="test_vector" type="irteusgl" />
    <test args="$(find euslisp)/test/matrix.l" pkg="euslisp" test-name="test_matrix" type="irteusgl" />
    <test args="$(find euslisp)/test/read-img.l" pkg="euslisp" test-name="test_read_img" time-limit="600" type="irteusgl">
  	
    </test>
    <test args="$(find euslisp)/test/graph.l" pkg="euslisp" test-name="test_graph" time-limit="600" type="irteusgl" />
  
  </launch>

test-irtrobot.launch
--------------------

.. code-block:: bash

  roslaunch euslisp test-irtrobot.launch



irteus robot model examples
---------------------------

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(full-body-ik)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/full_body_ik
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(full-body-ik :use-torso nil)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/full_body_ik_no_torso
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(full-body-ik :use-leg t)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/full_body_ik_use_leg
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(dual-arm-ik)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/dual_arm_ik
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(dual-manip-ik)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/dual_manip_ik
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(crank-motion)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/crank_motion
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(hand-grasp)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/hand_grasp
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(hanoi-arm)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/hanoi_arm
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl irteus/demo/demo.l "(particle)"

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/particle
  :width: 400

robots and object models
------------------------

.. code-block:: bash

  $ rosrun euslisp irteusgl models/irt-all-robots.l "(make-all-robots)"

.. image:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/irt_all_robots.png
  :width: 400

.. code-block:: bash

  $ rosrun euslisp irteusgl models/irt-all-objects.l "(make-all-objects)"

.. image:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-fuerte/lastSuccessfulBuild/artifact/doc/euslisp/html/_images/irt_all_objects.png
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
  
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'full-body-ik) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/full_body_ik.glc" pkg="euslisp" test-name="test_full_body_ik_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (defun\ full-body-ik-no-torso\ nil\ (full-body-ik\ :use-torso\ nil)) (setq\ demo-func\ #\'full-body-ik-no-torso) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/full_body_ik_no_torso.glc" pkg="euslisp" test-name="test_full_body_ik_no_torso_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (defun\ full-body-ik-use-leg\ nil\ (full-body-ik\ :use-leg\ t)) (setq\ demo-func\ #\'full-body-ik-use-leg) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/full_body_ik_use_leg.glc" pkg="euslisp" test-name="test_full_body_ik_use_leg_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'dual-arm-ik) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/dual_arm_ik.glc" pkg="euslisp" test-name="test_dual_arm_ik_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'dual-manip-ik) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/dual_manip_ik.glc" pkg="euslisp" test-name="test_dual_manip_ik_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'crank-motion) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/crank_motion.glc" pkg="euslisp" test-name="test_crank_motion_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'hand-grasp) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/hand_grasp.glc" pkg="euslisp" test-name="test_hand_grasp_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'hanoi-arm) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/hanoi_arm.glc" pkg="euslisp" test-name="test_hanoi_arm_demo" time-limit="600" type="irteusgl" />
    <test args="irteus/demo/demo.l  (setq\ demo-func\ #\'particle) $(find euslisp)/test/irteus-demo.l" launch-prefix="glc-capture --start --out=$(find euslisp)/build/particle.glc" pkg="euslisp" test-name="test_particle_demo" time-limit="600" type="irteusgl" />
    <test args="$(find euslisp)/build/full_body_ik.glc" pkg="jsk_tools" test-name="z_test_full_body_ik" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/full_body_ik_no_torso.glc" pkg="jsk_tools" test-name="z_test_full_body_ik_no_torso" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/full_body_ik_use_leg.glc" pkg="jsk_tools" test-name="z_test_full_body_ik_use_leg" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/dual_arm_ik.glc" pkg="jsk_tools" test-name="z_test_dual_arm_ik" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/dual_manip_ik.glc" pkg="jsk_tools" test-name="z_test_dual_manip_ik" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/crank_motion.glc" pkg="jsk_tools" test-name="z_test_crank_motion" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/hand_grasp.glc" pkg="jsk_tools" test-name="z_test_hand_grasp" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/hanoi_arm.glc" pkg="jsk_tools" test-name="z_test_hanoi_arm" time-limit="1000" type="glc_encode.sh" />
    <test args="$(find euslisp)/build/particle.glc" pkg="jsk_tools" test-name="z_test_particle" time-limit="1000" type="glc_encode.sh" />
  
    <test args="$(find euslisp)/test/all-robots-objects.l" pkg="euslisp" test-name="zz_test_all_robots_objects" time-limit="600" type="irteusgl" />
  </launch>

