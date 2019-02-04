^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.4 (2019-02-04)
------------------
* .travis.yml: run jsk_pr2eus tests in travis (`#599 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/599>`_ )

   * test/simple-client-cancel-test.l: add test to find `#567 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/567>`_ regression

* Revert "roseus: add :last-status-msg method for simple-action-client" (`#578 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/578>`_ )
* Revert "add test for subscribe object dispose" (`#525 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/525>`_ )
* Contributors: Kei Okada

1.7.3 (2019-02-01)
------------------
* roseus.cpp: when topic is subscribed twice, cleanup previous callback function (`#525 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/525>`_ )
  * add test for subscribe object dispose
  * if object is set to gentem symbol, we can not dispose them
  * test/test-subscribe-dispose.test : add test for dispose

* Fix typo in package.xml (`#598 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/598>`_ )
* test/add-two-ints-client.l: read request values from argument (`#596 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/596>`_ )
* This change enable to call
  ```
  $ rosrun roseus add_two_ints_client.l 4 5
  ```
  Closes `#581 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/581>`_
* roseus: add :last-status-msg method for simple-action-client (`#578 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/578>`_ )
* [roseus-util.l] Fixed typo in (one-shot-publish :after-stamp t) :tosec -> :to-sec (`#576 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/576>`_ )
* [roseus-util.l] fix comment typo: unction -> function (`#583 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/583>`_)
* roseus: reuse service server/client link on service call if connection is valid (`#593 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/593>`_ )
  * roseus.cpp: keep connection to server on persistent service call
  * test-add-two-ints.test: increase timelimit to 120 secs
  * test-service-callback.l: add test for calling unadvertised service
  * test-add-two-ints.l: add test code for persistent service call

* Fix frame-exists method (`#592 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/592>`_ )
  * Add code to wait for transform in test-frame-exists by @Affonso-Gui
  * Fix wrong function in the :frame-exists method. (Fix `#591 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/591>`_)

* roseus: setlocale to none (`#585 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/585>`_ )
* Contributors: Guilherme Affonso, Yuki Furuta, Kanae Kochigami, Kei Okada, Koga Yuya, Iori Yanokura

1.7.2 (2018-11-10)
------------------
* use PROJECT_SOURCE_DIR instaed of CMAKE_SOURCE_DIR for `catkin_make` (`#580 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/580>`_)
  * add debug message for COMPILE defun.c
  * use PROJECT_SOURCE_DIR instaed of CMAKE_SOURCE_DIR for `catkin_make`
* Contributors: Kei Okada

1.7.1 (2018-07-22)
------------------
* add melodic test (`#567 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/567>`_)
  * use rosrun roseus roseus test/test-namespace.l, instead of /usr/bin/env roseus
* update function using new defun function (`#569 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/569>`_)
  * CHNAGED EusLisp defun() arguments, see https://github.com/euslisp/EusLisp/pull/300
  * force generate defun.h header file
  * use euslisp(9.24)'s new defun api roseus.cpp taht takes doc as argument, remove _defun
  * add documentation string to defun functions
  * add documentation string to defun functions, (roseus_c_util.c uses NULL because this is not exported functions
* [roseus.l] add length check for argument when searching __log:=t (`#568 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/568>`_)
* Contributors: Kei Okada, Yohei Kakiuchi

1.7.0 (2018-07-11)
------------------
* (ros::tf-transform->coords) failed with with geometry_msgs::Transform (`#563 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/563>`_)
  * fix when tf-transform->coords receives geometry_msgs::Transform
  * [test/eustf.l] Add test to check (ros::tf-transform->coords) with geometry_msgs::Transform
* [roseus-utils.l]fix eusobj->marker-msg (`#555 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/555>`_)
  * load dependent packages if msg/srv is not found in ros::load-ros-manifest
  * add ros::rospack-depends
  * roseus is always have roseus/ros/roseus
  * add debug message when loading manifest/msg/srv files, also more message when we need to avoid using load-ros-manifest
  * INDIGO: https://github.com/jsk-ros-pkg/jsk_roseus/issues/554
    if pkg without msg, need to return no-msg-package
    JADE/KINETIC: https://github.com/jsk-ros-pkg/jsk_robot/issues/823
    package without msg does not have manifest.l
  * fix find-load-msg-path, use dirs ~/share instaed of ~/share/roseus, it does not change logic

* do not load manifest.l when we have source tree, but does not have manifest.l because of missing msg/srv (`#554 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/554>`_)
* add test to check :connection-header (`#540 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/540>`_)
  * roseusp.cpp : add :connection-header method
  * add test to check :connection-header

* fix dead locking on accessing rosparam in timer callback (`#557 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/557>`_)
  * test-mark-lock.l: add function using mark_lock
  * roseus: remove mutex_lock / unlock
  * roseus: add test code for dead locking of mark_lock

* add test code for [unexpected behavior if message has property `name`] #508 (`#509 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/509>`_)
  * add test code for #508

* add ros::duration-sleep (`#549 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/549>`_)
  * test/test-roseus.l : add test for ros::duration-sleep
  * roseus.cpp/roseus.l : add ros::duration-sleep

* Support both AcitonGoal and Goal in :send-goal (`#546 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/546>`_)
  * test/simple-client-test.l: fix typo returns -> return
  * actionlib.l:send-goal : send-goal accepts both ActionGoal and Goal, where Python and C only takes Goal, but original roseus takes ActoinGoal, here we make ActionGoal when Goal is passed as python/c client
  * send SimpleGoal, not SimpleActionGoal

* roseus.cpp: use boost::shared_ptr for ros::Rate (`#553 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/553>`_)
* add test to check numbers in node name (#536) (`#552 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/552>`_)
  * Allow numbers on ros::roseus node name
  * add test to check numbers in node name (#536)

* Fix roseus test error, see https://github.com/jsk-ros-pkg/jsk_roseus/pull/545#issuecomment-349224047 (`#551 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/551>`_)
  * add more messages, fix for roslaunch 1.12.8, due to //github.com/ros/ros_comm/issues/1097,
  * print error message when test-rosmaster failed
  * relax hzerror in test-anonymous.test,  ex, https://travis-ci.org/jsk-ros-pkg/jsk_roseus/jobs/313841319
  * test-roseus.l: test-master, add comment on tests
  * test-roseus.l: fix test error on test-master
    ros::get-uri returns http://localhost:11311, not http://localhost:11311/, also accept arbitrary order in ros::get-nodes, ros::get-topics tests


* Contributors: Yuki Furuta, Guilherme Affonso, Kei Okada, Naoki Hiraoka, Iori Yanokura

1.6.3 (2017-09-08)
------------------
* Fix ros::get-namesapce (`#533 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/533>`_)
  * use ros::names::clean to get sanitized namespace string
  * add test for ros::get-namesapce

* package without msg does not have manifest.l, so skip loading that without  ros::ros-error (`#539 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/539>`_)
  * on jade/kinetic, package without msg does not have manifest.l, so users need to change (load-ros-manifest) target
  * add test to check https://github.com/jsk-ros-pkg/jsk_roseus/pull/537 / https://github.com/jsk-ros-pkg/jsk_robot/issues/823
* [roseus][roseus.cpp] check ros::ok() in ros::spin (`#531 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/531>`_ )
* [roseus/euslisp/actionlib.l] fix :wait-for-result is too slow (`#528 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/528>`_)
* Contributors: Kei Okada, Yohei Kakiuchi

1.6.2 (2017-06-21)
------------------
* CMakeLists.txt: find_package jskeus and add euslisp/jskeus to DEPENDS in CMakeLists.txt to get euslisp/jskeus version (`#514 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/514>`_)
* [roseus_utils.l] fix make-camera-from-ros-camera-info-aux (`#526 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/526>`_)
* skip test/test-genmsg.catkin.test (`#518 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/518>`_)
* if goal is overridden from different instance in same roseus process, actionlib do not return from :wait-for-result. (updated
  version of #519) (`#521 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/521>`_)
  * actinlib.l : add :name-space method to simple-action-server
  * print warn if :wait-for-result ends with preempted
  * add test-simple-client-cancel.test for https://github.com/start-jsk/jsk_apc/issues/2106
  * set queue of status/result/feedback cb from 1 to 8, to get old results, also keep action-client to global list and if result is not yours, look client from list
  * actionlib.l : fix error when (send comm-state :action-goal) do not exists
  * use gentemp to bound object, to find from do-symbols
  * add test-client-dispose
* roseus/euslisp/roseus-utils.l: update make-camera-from-ros-camera-info (`#517 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/517>`_)
* CMakeLists.txt: use grep package.xml when git --tags did not retun any message (it happens in build farm) (`#516 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/516>`_)
* tell full path of roseus diretory when load roseus.l ... (`#515 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/515>`_)
* CMakeLists.txt: find_package jskeus and add euslisp/jskeus to DEPENDS in CMakeLists.txt to get euslisp/jskeus version (`#514 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/514>`_)
* Contributors: Kei Okada, YoheiKakiuchi

1.6.1 (2017-03-15)
------------------
* remove compiler warning from roseus.cpp (`#510 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/510>`_)
  * [hydro] do not eliminate -> warning: deleting object of polymorphic class type ‘tf2_ros::BufferClient’ which has non-virtual destructor might cause undefined behaviour [-Wdelete-non-virtual-dtor]
* [roseus][eustf.l] fix: pass :init args (`#506 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/506>`_)
* add kinetic test (`#505 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/505>`_)
  * test-geneus.test : use rosrun roseus roseus to run test code
  * roseus/CMakeLists.txt : add -DNDEBUG option, see https://github.com/jsk-ros-pkg/jsk_planning/pull/49#issuecomment-280302156
* test/test-tf.test: not sure why but test-tf fails within travis, but works in droplet 2G/2CPU (`#499 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/499>`_)
* default queue size of subscribe/advertise is 1, add this information to documentation (`#493 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/493>`_)
* Fix `#417 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/417>`_ (`#486 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/486>`_)
  * [roseus/roseus.cpp] fix: segfault when no response is returned on service callback
  * [roseus] add test-service-callback.test
  * [roseus/roseus.cpp] return false when service callback returns invalid response
  * [roseus/roseus.cpp] use C++ bool for return value
* add aarch64 for arm processors (`#484 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/484>`_)
* [roseus] add example of actionlib feedback (`#479 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/479>`_)
  * [roseus/test/fibonacci-client.l] remove unnecessary new lines.
  * [roseus/test/fibonacci-client.l] add feedback callback.
  * [roseus/test/fibonacci-server.l] remove unnecessary new lines.
  * [roseus/test/fibonacci-client.l] fix correspondence of brackets.
  * [roseus/test/fibonacci-server.l] publish feedback of fibonacci action in loop.
* Contributors: Kei Okada, Masaki Murooka, Yuki Furuta

1.6.0 (2016-10-02)
------------------
* Support private/under-namespace topic name in roseus client
  Node            nRelative (default)      Global          Private
  /node1          bar -> /bar             /bar -> /bar    ~bar -> /node1/bar
  /wg/node2       bar -> /wg/bar          /bar -> /bar    ~bar -> /wg/node2/bar
  /wg/node3       foo/bar -> /wg/foo/bar  /foo/bar -> /foo/bar    ~foo/bar -> /wg/node3/foo/bar
* Fix test to fail when no message came
* when pkg is target package do not need to find_package, just to set SOURCE_PREFIX, this will solve https://github.com/jsk-ros-pkg/jsk_robot/issues/597
* Remove definition of unused variables
* [roseus-utils.l] fix dump-pointcloud-to-pcd-file file
* [roseus/test/param-test.l] fix: param test for cache
* [roseus/roseus.cpp] fix typo: ros::get-param-cashed -> ros::get-param-cached
* [roseus/roseus.cpp] add ros::delete-param
  [roseus/test/param-test.l] add test for ros::delete-param
* [roseus/CMakeLists.txt] remove coreutils from DEPENDS
* [roseus/package.xml] add coreutils to build_depend
* [roseus/CMakeLists.txt] add CATKIN_ENABLE_TESTING section for testing
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi, Yuki Furuta

1.5.3 (2016-05-28)
------------------

1.5.2 (2016-05-28)
------------------
* Support OSX (again..)

  * Do not use EUSDIR env in generate_eusdoc cmake macro for osx (`#448 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/448>`_)
  * Find euslisp include directories on OS X (`#448 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/448>`_)
  * re-define get_string for osx (`#455 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/455>`_)
  * Set correct EUSDIR for roseus exe on OS X (`#449 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/449>`_)

* Set xvfb as test_depend and stop installing it before_script (`#443 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/443>`_)
  Modified:
  - .travis.yml
  - roseus/package.xml

* Contributors: Kei Okada, Kentaro Wada

1.5.1 (2016-04-22)
------------------
* Fix generating Euslisp ROS message with catkin_tools 0.4.x
  Modified:
  - roseus/cmake/roseus.cmake
* Contributors: Kentaro Wada

1.5.0 (2016-03-20)
------------------

* support dictionary for set-param

  * roseus.cpp: SET_ROS_PARAM clean up error message
  * roseus.cpp: fix typo, unkown -> unknown
  * roseus.cpp: (ros::set-param): support to set directory
  * test/param-test.l : add test for set-param
  * test/param-test.l: display parameters

* misc updates

  * cmake/roseus.cmake: quiet find_pakcage, this may fail for the first time
  * test/test-genmsg.sh: add include_directories(${catkin_INCLUDE_DIRS})
  * [roseus] Retry 3 times actionlib test

* image conversion

  * [roseus/euslisp/roseus-utils.l] add image conversion to ros msg
    [roseus/test/test-roseus.l] add test for image conversion
    [roseus/test/test-roseus.test] use virtual display for test with viewer
    [.travis.yml] install xvfb before_install to launch X server on test

* Contributors: Furushchev, Kei Okada, Ryohei Ueda

1.4.1 (2015-11-25)
------------------
* euslisp/actionlib.l

  * euslisp/actionlib.l: set queue_size following to action_server_imp.h and action_client_imp.h `#396 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/396>`_ (https://github.com/ros/actionlib/blob/indigo-devel/include/actionlib/server/action_server_imp.h#L121, https://github.com/ros/actionlib/blob/indigo-devel/include/actionlib/client/action_client.h#L210)
  * euslisp/actionlib.l : wait-for-goal: returns nil when no goal is found (https://github.com/jsk-ros-pkg/jsk_roseus/pull/410)
  * euslisp/actionlib.l : goal_id must be unique : set goal_id to use current nsec

* roseus/utils

  * [roseus/euslisp/roseus-utils.l] fix typo message type

* cmake/get_all_depends.py

  * hot fix until https://github.com/jsk-ros-pkg/geneus/pull/42 has released

* test

  * test-simple-client-*500.test: add test to run simple-client with high-speed status
  * test/test-timer.l: surpress output message
  * test/test-tf.l: surpress output message
  * test/test-actionlib.l: surpress output message
  * test/test-add-two-ints.l use ros-info instead of warning-message to suppress message
  * test/add-two-ints-{client,server}.l use ros-info instead of warning-message to suppress the message
  * 00x-fibonacci-test-{1,2}.launch: fibonacci\_{server,client}.py is not longer avilable, use fibonacci\_{server,client}
  * test/test-genmsg.catkin.test: disable --remove-message test, which does not work on paralllel execution
  * test/test-genmsg: add debug message
  * test/test-actionlib.l: :wait-for-results returns nil when no goal has been sent
  * test/test-actionlib.l: simple-action-client must be a global variable
  * test/test-actionlib.l: add test to run send-goal twice with difference client instance
  * roseus/test/test-actionlib.test: re-enable test-actionlib.test, which is disabled since groovy

* Contributors: Yuki Furuta, Kamada Hitoshi, Kei Okada, Kentaro Wada, Ryohei Ueda, Shunichi Nozawa

1.4.0 (2015-11-03)
------------------
* Fix bugs in bool array (https://github.com/jsk-ros-pkg/geneus/issues/38)

  * [test/test-geneus.l] use list for bool array
  * [test/test-geneus.l] add test for time/duration/object array
  * [tes/test-geneus.ll] Add test for VariableArray. Currently, bool_data fails because of bug reported in https://github.com/jsk-ros-pkg/geneus/issues/38
  * [test/test-geneus.l, roseus/test/test_geneus_send_msgs.py] Add test for FixedArray.msg and this test will pass currently.
  * [roseus/msg/FixedArray.msg, roseus/msg/VariableArray.msg] Add VariableArray msg and add bool field to Fixedarray.msg

* New Features

  * [roseus] Add ros::rospack-plugins function. It is equivalent to
  `rospack plugins ...`
  ```lisp
  (ros::rospack-plugins "nodelet" "plugin")
  =>
  (("laser_proc" . "/opt/ros/hydro/share/laser_proc/nodelets.xml") ("velodyne_driver" . "/opt/ros/hydro/share/velodyne_driver/nodelet_velodyne.xml") ("yocs_velocity_smoother" . "/opt/ros/hydro/share/yocs_velocity_smoother/plugins/nodelets.xml") ("jsk_perception" . "/home/lueda/ros/hydro/src/jsk-ros-pkg/jsk_recognition/jsk_perception/jsk_perception_nodelets.xml") ("image_rotate" . "/home/lueda/ros/hydro/src/image_pipeline/image_rotate/nodelet_plugins.xml") ("stereo_image_proc" . "/home/lueda/ros/hydro/src/image_pipeline/stereo_image_proc/nodelet_plugins.xml") ("depth_image_proc" . "/home/lueda/ros/hydro/src/image_pipeline/depth_image_proc/nodelet_plugins.xml") ("kobuki_bumper2pc" . "/opt/ros/hydro/share/kobuki_bumper2pc/plugins/nodelet_plugins.xml") ("kobuki_safety_controller" . "/opt/ros/hydro/share/kobuki_safety_controller/plugins/nodelet_plugins.xml") ("naoqi_sensors" . "/home/lueda/ros/hydro/src/ros_naoqi/naoqi_bridge/naoqi_sensors/naoqicamera_nodelet.xml") ("velodyne_pointcloud" . "/opt/ros/hydro/share/velodyne_pointcloud/nodelets.xml") ("pointcloud_to_laserscan" . "/home/lueda/ros/hydro/src/perception_pcl/pointcloud_to_laserscan/nodelets.xml") ("openni2_camera" . "/opt/ros/hydro/share/openni2_camera/openni2_nodelets.xml") ("resized_image_transport" . "/home/lueda/ros/hydro/src/jsk-ros-pkg/jsk_recognition/resized_image_transport/nodelet.xml") ("image_proc" . "/home/lueda/ros/hydro/src/image_pipeline/image_proc/nodelet_plugins.xml") ("uvc_camera" . "/opt/ros/hydro/share/uvc_camera/nodelet_uvc_camera.xml") ("openni_camera" . "/opt/ros/hydro/share/openni_camera/openni_nodelets.xml") ("yocs_cmd_vel_mux" . "/opt/ros/hydro/share/yocs_cmd_vel_mux/plugins/nodelets.xml") ("pcl_ros" . "/home/lueda/ros/hydro/src/perception_pcl/pcl_ros/pcl_nodelets.xml") ("prosilica_camera" . "/home/lueda/ros/hydro/src/prosilica_driver/prosilica_camera/plugins/nodelet_plugins.xml") ("jsk_topic_tools" . "/home/lueda/ros/hydro/src/jsk-ros-pkg/jsk_common/jsk_topic_tools/jsk_topic_tools_nodelet.xml") ("jsk_pcl_ros" . "/home/lueda/ros/hydro/src/jsk-ros-pkg/jsk_recognition/jsk_pcl_ros/jsk_pcl_nodelets.xml") ("image_view" . "/home/lueda/ros/hydro/src/image_pipeline/image_view/nodelet_plugins.xml") ("nodelet_tutorial_math" . "/opt/ros/hydro/share/nodelet_tutorial_math/nodelet_math.xml") ("imagesift" . "/home/lueda/ros/hydro/src/jsk-ros-pkg/jsk_recognition/imagesift/nodelet.xml"))
  ```
* Warning Message

  * [roseus/roseus.cpp] remove trivial error message from get-num-publishers
  * [roseus/euslisp/actionlib.l: add warning message when action server is not found

* Misc

  * [roseus/cmake/roseus.cmake] run message generation at build form for pr2eus
  * [roseus] Not import no used module in get_all_depends.py (#337)

* Contributors: Yuki Furuta, Kamada Hitoshi, Kei Okada, Kentaro Wada, Ryohei Ueda, Shunichi Nozawa

1.3.9 (2015-09-14)
------------------
* roseus.cpp: add ros::create-timer function
* Contributors: Kei Okada, Ryohei Ueda

1.3.8 (2015-09-12)
------------------
* [roseus] Add test to read ros parameter with default value 1000 times
* [roseus] Use COPYOBJ instead of copyobj to copy object of default
  parameter in ros::get-param
* fix ros::resolve-path returns nil for non existing package name
* add test for ros::resolve-path
* [euslisp/roseus.l] compile when loaded as package://
* [euslisp/roseus.l] fix roseus-add-files to use normal compile-file-if-src-newer
* [test/test-compile-message.l] add test for compiling message
* Contributors: Kei Okada, Ryohei Ueda, Yohei Kakiuchi

1.3.7 (2015-08-18)
------------------
* geneus stuff

  * [cmake/get_all_depends.py] hydro releaes still uses 2.2.2, so we need to update pkg_map
  * [cmake/roseus.cmake] display eus-related package version
  * [cmake/roseus.cmake] call find_package  to get ${_pkg}_PREFIX
  * [cmake/roseus.cmake] fix for get_all_depends in installed space
  * [cmake/roseus.cmake] Set CMAKE_PREFIX_PATH to run generate all deps
  * [cmake/roseus.cmake] Add condition for roseus_SOURCE_PREFIX when building roseus
  * [cmake/roseus.cmake] Add macro(_package_depends_impl) in roseus.cmake
  * [cmake/get_all_depends.py] Add cmake/get_all_depends.py to get all implicit depends

* marker conversion
  * [euslisp/roseus-utils.l] fix eusobj->marker-msg 's check body type
  * [euslisp/roseus-utils.l] remove debug code (marker-msg->shape)

* test codes
  * [test/test-roseus.l] add test for irtpointcloud
  * [test/test-roseus.l] add test code for marker message <-> eus object conversion function in euslisp/roseus-utils.l
  * [test/test-genmsg.sh, test/test-genmsg.catkin.test] check after remove messages in devel/share/roseus/ros
  * [test/test-genmsg.sh] add test to check if messages in roseus is generated
  * [roseus/test/test-rosues.l] make-random-pointcloud is only available on jskeus 1.0.9

* build system
* [roseus/CMakeLists.txt] somehow regex in if statemet must be double quated?
* [roseus/cmake/roseus.cmake] Unset DISPLAY environmental variable when generating eusdoc to avoid init-xwindow error
* [roseus] Add .gitignore

* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Yohei Kakiuchi, Yuto Inagaki

1.3.6 (2015-06-11)
------------------
* [CMakeLists.txt] add catkin_INCLUDE_DIRS, this fixes #317
* [roseus] Add NO_GENERATE_EUSDOC environmental variable to disable
  generation of eusdoc
* Contributors: Kei Okada, Ryohei Ueda

1.3.5 (2015-05-15)
------------------
* [roseus.cpp] remove error message in get-topic-subscriber
* [roseus.cpp] add more documentations
* [cmake/roseus.cmake] update generate_eusdoc for installed functions\n\n this requires https://github.com/euslisp/EusLisp/pull/112
* [cmake/roseus.cmake] do not raise error when geneus doc failed
* [euslisp/{eustf.l, roseus-utils.l, roseus.l}] add more documenations
* [roseus.cpp] is fix error message, You must call ros::init() -> (ros::roseus "name")
* [roseus/CMakeLists.txt] add compiler option for C to suppress looking-up undefined symbol when linking using Clang compiler
* [roseus/eustf.cpp] undef duplicated macros defined in standard library and in euslisp
* [roseus.cpp] remove error message meanless in get-topic-publisher
* Contributors: Yuki Furuta, Kei Okada, Yuto Inagaki

1.3.4 (2015-05-03)
------------------
* [roseus.cpp] add get-host, get-nodes, get-port, get-uri, get-topics, from http://docs.ros.org/indigo/api/roscpp/html/master_8h.html
* [euslisp/roseus-utils.l] support bodyset object
* [euslisp/roseus-utils.l] support random color
* [euslisp/roseus-utils.l] support object with :glvertices
* [jsk_roseus] Parallelize generate-all-msg-srv
* Contributors: Kei Okada, Ryohei Ueda

1.3.3 (2015-04-29)
------------------
* [roseus/cmake/roseus.cmake] need to know roseus exeutable path when compile within same workspace
* [roseus/CMkeLists.txt] in some cases, rosversion tf2_ros did not resspond the results, use tf2_ros_VERSION, since this is only for old tf2, so we can remove this
* Contributors: Kei Okada

1.3.2 (2015-04-28)
------------------
* [cmake/roseus.cmake] use ${PROJECT_NAME}_generate_messages_eus_all_target for depend to eusdoc
* Contributors: Kei Okada

1.3.1 (2015-04-26)
------------------
* [cmake/roseus.cmake] fix for package only with action
* [roseus/test/roseus.cmake] check package only action messages, (jsk_demo_common)
* Contributors: Kei Okada

1.3.0 (2015-04-24)
------------------

* add generate_eusdoc

  * [roseus/cmake/roseus.cmake] depends on install_roseus for doc generation
  * [roseus/CMakeLists.txt] generate eus-docs
  * [roseus/cmake/roseus.cmake] add generate_eusdoc macro

* CMakeLists.txt

  * [roseus/CMakeLists.txt] use add_custom_target to copy roseus to   bin

* roseus.cmake

  * [cmake/roseus/roseus.cmake] fix for msg in workspace using {$msg}_SOURCE_PREFIX
  * do not raise error for old catkin

* convert unit8[] as string https://github.com/jsk-ros-pkg/geneus/issues/14

  * [test/test_geneus] add test for fixed length data
  * [test-genmsg.sh] compile with -j1 and -l1, unset MAKEFLAGS  https://github.com/catkin/catkin_tools/pull/85
  * [roseus] fix test for treating uint8[] as string

* [roseus] add test-anonymous for `#179 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/179>`_
* Contributors: Yuki Furuta, Kei Okada

1.2.6 (2015-02-21)
------------------
* [test-genmsg.sh] fix for latest source code
* [CMakeLists.txt] create symlink from share/roseus -> ../../bin/roseus
* [test-genmsg.sh] fix typo rosun -> rosrun
* [test/test-genmsg.sh] add test for 'manifest should have all depends packages'
* [test/test-genmsg.sh] remove rosbuild settings
* [roseus] Install roseus binary to share directory
* [generate-all-msg-srv] fix msg gen
* Contributors: Yuki Furuta, Kei Okada, Yuto Inagaki

1.2.5 (2015-02-13)
------------------
* [roseus.cmake] add more condition
* [roseus] Add class to synchronize multiple topics with the same timestamp like message_filters
* Contributors: Ryohei Ueda, Kei Okada

1.2.4 (2015-02-12)
------------------
* do not run upstream message generation on buildfirm
* fir for generating manifest for packages does not have depends
* add test code for geneus
* more fix to generate-all-msg-srv
* fit for generating msgs
* add target package those who does not have msg files
* [roseus] generate-all-msgs-srv.sh fix for new geneus package
* roseus messages under home-dir is nolonger supported
* [roseus] add more debug messages ros message generation
* [roseus] test/test-genmsg.sh, fix typo start-from -> start-with for catkin-tools
* Contributors: Kei Okada

1.2.3 (2015-02-02)
------------------
* find package if not messages path is not found
* [roseus] Fix typo
* euslisp is now non-catkin package

1.2.2 (2015-01-27)
------------------
* do not compile message if it is already installed

1.2.1 (2015-01-27)
------------------
* install generated messages

1.2.0 (2015-01-26)
------------------
* [roseus] If user return invalid instance in service callback, print error message
* use EUSDIR insted of using rospack find euslisp
* Contributors: Ryohei Ueda, Kei Okada

1.1.33 (2015-01-26)
-------------------
* fix wrong all_generate_message_eus target

1.1.32 (2015-01-26)
-------------------
* generate all roseus messages on buildfirm

1.1.31 (2015-01-23)
-------------------
* add dynamic_reconfigure
* fix to use catkin-tools
* remove old manifest.xml, fully catkinize
* use originl source (node rosmake proxy package) for euslisp
* add new macro, generate_all_roseus_message() to generate all dependency msgs using new geneus written by python
* enable alpha when converting eus object to ros marker
* [roseus] Fix error of VERSION_LESS around TF2_ROS_VERSION
* Contributors: Ryohei Ueda, Kei Okada, Yusuke Furuta

1.1.30 (2015-01-14)
-------------------
* use -L to find symlinked irteusgl

1.1.29 (2014-12-27)
-------------------
* check it euslisp provide euslisp_INCLUDE_DIR

1.1.28 (2014-12-26)
-------------------
* simplify function
* add compare function for ros::time
* Contributors: Chi Wun Au

1.1.27 (2014-12-20)
-------------------
* update body's worldcoords before using its faces
* add logger and level key param to ros::roseus
* fix typo of ros::coords->pose
* add :anonymous to ros::roseus
* add set_logger_level func
* modified typo ros::rosinfo => ros::ros-info
* add warning if id is set
* update param-test.l for testing parameter handling by roseus
* add code for reading dictionary type parameter to roseus

1.1.26 (2014-11-10)
-------------------
* Add utility function to set dynamic_reconfigure parameter
* Contributors: Ryohei Ueda

1.1.25 (2014-10-10)
-------------------

1.1.23 (2014-09-24)
-------------------

1.1.22 (2014-09-04)
-------------------
* install roseus to global bin, fixed #146
* fix bracket mathing in roseus-utils.l
* call error when package:// is not found, fix typo, see #140
* Contributors: Kei Okada, Masaki Murooka

1.1.21 (2014-06-30)
-------------------

1.1.20 (2014-06-29)
-------------------
* roseus_c_util.c : remove compile_warnings
* test-genmsg.sh: add roscpp to CATKIN_DEPENDS
* test-genmsg.sh: catkin_make with --make-args VERBOSE=1
* test-genmsg.sh/test-genmsg.catkin.test : check #120 situation
* roseus.cpp : support reconnection of service when persist is set true
* Contributors: Ryohei Ueda, Kei Okada

1.1.19 (2014-06-11)
-------------------
* (#112,#113) fix service persist without keyward
  ros::service-call (name value &optional (persist nil))
* Contributors: Ryohei Ueda, Kei Okada

1.1.18 (2014-05-16)
-------------------

1.1.17 (2014-05-11)
-------------------

1.1.16 (2014-05-11)
-------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------
* add hasHeader for roscpp >= 1.11.1
* fix typo in install roseus
* Contributors: Kei Okada

1.1.13 (2014-05-06)
-------------------
* add more message when install roseus
* Contributors: Kei Okada

1.1.12 (2014-05-06)
-------------------

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03)
-------------------

1.1.9 (2014-05-03)
------------------
* add debug message when install roseus
* Contributors: Kei Okada

1.1.8 (2014-05-02)
------------------
* create symlink in global/bin/roseus
* Contributors: Kei Okada

1.1.7 (2014-04-28)
------------------

1.1.6 (2014-04-28)
------------------

1.1.5 (2014-04-27)
------------------

1.1.4 (2014-04-25)
------------------
* check msg file udder CMAKE_PREFIX_PATH (#68)
* (#31) use 120 as wait-for-transform
* Contributors: Kei Okada

1.1.3 (2014-04-14)
------------------
* add rosdnoe to depends(#64)
* Contributors: Kei Okada

1.1.0 (2014-04-07)
------------------
* add geneus package that generate ros message for euslisp
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) copy jsk_roseus for one workspace and remove build on rosbuild
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add rich test for euslisp message generation, remove scripts and generate them from one shell script.
  * one workspace/separated workspace
  * add several dependency
  * action messages generation
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add scripts to test geneus more
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) check if test the message has created or not by simple roseus program, add euslisp test rather than cpp test code
* (`#32 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/32>`_) add test-genmsg, test message generation on catkin and rosbuild
* add check delay of lookuptransform
* add checking delay of tf return
* Contributors: Ryohei Ueda, Yohei Kakiuchi, Kei Okada

1.0.4 (2014-03-31)
------------------
* fix for catkin environment
* set euslisp_PACKAGE_PATH for both devel and installed
* switch from svnversion to git rev-parse --short HEAD
* removed debug messages
* Contributors: Ryohei Ueda, Kei Okada

1.0.3 (2014-03-29)
------------------
* catkin.cmake add rostest to find_package
* `#14 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/14>`_: depend roseus message generation on python message generation.
  in roseus.cmake, do not take into account the dependencies between messages
  and packages and just depends roseus message generation on python message generation.
  The 1st reason is the difference between hydro and groovy. On groovy, genmsg
  does not craete the targets of foo_generate_messages_py
  which are already compiled, I mean the packages installed by apt.
  The 2nd reason is that roseus message generation utilizes rospy and it requires
  for rospy messages to be available. So this dependencies are required.
  Namely, the dependency will be like this:
  parent_pkg
  +-child_pkg
    +-grandchild_pkg
      +-grandchild_pkg_generate_messages_py
        +-euslip targets for grandchild_pkg
* Contributors: Ryohei Ueda
* roseus/test/test-tf.test: tf2_buffer_server output to screen

1.0.2 (2014-03-28)
------------------
* roseus.cmake: remove debug code
* roseus/test/test-add-two-ints.l: reduce test time
* Contributors: Kei Okada

1.0.1 (2014-03-27)
------------------
* roseus: add version numeber to 1.0.0
* Contributors: Haseru Chen, Shunnich Nozawa, Yuki Furuta, Kei Okada, Yuto Inagaki, Manabu Saito, kazuto Murase, Yohei Kakiuchi, Eisoku Kuroiwa, Ryohei Ueda, Hiroyuki Mikita
