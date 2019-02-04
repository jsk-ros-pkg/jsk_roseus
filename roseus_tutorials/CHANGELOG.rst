^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roseus_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.4 (2019-02-04)
------------------

1.7.3 (2019-02-01)
------------------
* use ar_pose_alvar instead of ar_pose for kinetic (`#582 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/582>`_ )
* Contributors: Yuki Furuta

1.7.2 (2018-11-10)
------------------

1.7.1 (2018-07-22)
------------------
* run depends on pr2eus has circular dependencies, so removed from package.xml (`#569 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/569>`_)
* Contributors: Kei Okada

1.7.0 (2018-07-11)
------------------

1.6.3 (2017-09-08)
------------------

1.6.2 (2017-06-21)
------------------

1.6.1 (2017-03-15)
------------------

1.6.0 (2016-10-02)
------------------

1.5.3 (2016-05-28)
------------------
* refactor roseus_tutorials (`#436 <https://github.com/jsk-ros-pkg/jsk_roseus/issues/436>`_)

  * [roseus_tutorials/launch/tabletop-object-detector.launch] fix camera namespace
  * [roseus_tutorials/src/tabletop-object-detector.l] enable on recentversion
  * [roseus_tutorials/src/tabletop-object-detector.l] publish objects tf; add param to switch if publish objects tf
  * [roseus_tutorials/CMakeLists.txt] fix typo
  * [roseus_tutorials] add test for tabletop object detection
  * [roseus_tutorials] add config directory to install
  * [roseus_tutorials] move rviz config files to config directory
  * Contributors: Kei Okada, Yuki Furuta

1.5.2 (2016-05-28)
------------------

1.5.1 (2016-04-22)
------------------

1.5.0 (2016-03-20)
------------------

1.4.1 (2015-11-25)
------------------
* add more depends
* roseus_tutorials : use face_detection in opencv_apps
* Contributors: Kei Okada

1.4.0 (2015-11-03)
------------------

1.3.9 (2015-09-14)
------------------

1.3.8 (2015-09-12)
------------------

1.3.7 (2015-08-18)
------------------

1.3.6 (2015-06-11)
------------------

1.3.5 (2015-05-15)
------------------

1.3.4 (2015-05-03)
------------------

1.3.3 (2015-04-29)
------------------

1.3.2 (2015-04-28)
------------------

1.3.1 (2015-04-26)
------------------

1.3.0 (2015-04-24)
------------------

1.2.6 (2015-02-21)
------------------

1.2.5 (2015-02-13)
------------------

1.2.4 (2015-02-12)
------------------

1.2.3 (2015-02-02)
------------------

1.2.2 (2015-01-27)
------------------

1.2.1 (2015-01-27)
------------------

1.2.0 (2015-01-26)
------------------

1.1.33 (2015-01-26)
-------------------

1.1.32 (2015-01-26)
-------------------

1.1.31 (2015-01-23)
-------------------
* remove old manifest.xml, fully catkinize
* Contributors: Kei Okada

1.1.30 (2015-01-14)
-------------------

1.1.29 (2014-12-27)
-------------------

1.1.28 (2014-12-26)
-------------------

1.1.27 (2014-12-20)
-------------------
* update publish-marker.launch
* Enable to change root-frame-id
* Use argument for checkerboard detector. Enable to set group.
* Replace deprecated kinect_color_filter by hsi_color_filter in jsk_pcl_ros
* update vision-action-example3 to use checkerboard_detector
* Contributors: Kei Okada, Shunichi Nozawa, Yuto Inagaki

1.1.26 (2014-11-10)
-------------------

1.1.25 (2014-10-10)
-------------------

1.1.24 (2014-09-24 11:56:16)
----------------------------

1.1.23 (2014-09-24 11:56:02)
----------------------------

1.1.22 (2014-09-04)
-------------------

1.1.21 (2014-06-30)
-------------------

1.1.20 (2014-06-29)
-------------------

1.1.19 (2014-06-11)
-------------------
* roseus_tutorials/launch/usb-camera.launch: rename camera_node -> uvc_camera_node for deb package
* moving params from constant to args in tabletop-object-detector.launch
* Contributors: Hiroaki Yaguchi, Kei Okada

1.1.18 (2014-05-16)
-------------------

1.1.17 (2014-05-11 13:27)
-------------------------

1.1.16 (2014-05-11 03:23)
-------------------------

1.1.15 (2014-05-10)
-------------------

1.1.14 (2014-05-09)
-------------------

1.1.13 (2014-05-06 15:36)
-------------------------

1.1.12 (2014-05-06 03:54)
-------------------------

1.1.11 (2014-05-04)
-------------------

1.1.10 (2014-05-03 10:35)
-------------------------

1.1.9 (2014-05-03 09:30)
------------------------

1.1.8 (2014-05-02)
------------------

1.1.7 (2014-04-28 14:29)
------------------------

1.1.6 (2014-04-28 03:12)
------------------------

1.1.5 (2014-04-27)
------------------

1.1.4 (2014-04-25)
------------------
* update tabletop detector
* Contributors: Yohei Kakiuchi

1.1.3 (2014-04-14)
------------------

1.1.2 (2014-04-07 23:17)
------------------------

1.1.1 (2014-04-07 09:02)
------------------------

1.1.0 (2014-04-07 00:52)
------------------------

1.0.4 (2014-03-31)
------------------

1.0.3 (2014-03-30)
------------------

1.0.2 (2014-03-28)
------------------

1.0.1 (2014-03-27)
------------------
* roseus_tutorials: comment out many packages that does not have entry for groovy
* deprecate aques_talk
* #5: remove cmvision, no longer available
* #5: remove ar_pose because it's out of date and not maintained
* add comment for kinect
* debug eus-pointcloud-example.l
* add eus-pointcloud-example.l (how to publish PointCloud2 and how to dump or load)
* update for passthrough naming
* add name to pcl_manager
* udpate
* update topic variable name
* use ROS_DISTRO to find haarcascade file
* add comments for bounding box
* add automatically update
* minor update
* add roi-reconfigure-call.l
* update name remapping
* frame_id became argument in usb-camera.launch
* switch image_proc node to image_proc nodelet launching file
* add argument to specify color name
* replace openni -> camera because camera topic is used in kinect_color_filter.launch
* add camera_info_url argument to usb-camera.launch
* add calibration data file of Logicool Orbit camera
* device param of usb-camera.launch became arg
* rename frame_id which is reported at [#241]
* add face->marker-msg example
* add line_strip example
* suppor rpy style in relative_pose, status:closed #139
* add function start-subscribe to subscribe-pointcloud.l
* add sample for detecting image template
* add comments, thanks google accounts??
* add launch_objectdetection arguments for publish /ObjectDetection
* add parameter: convert_to_base_link
* change: kinect topic name
* add arguments
* add parameter transform_table
* fix: face detection parameter for fuerte
* add kinect_color_filter.launch
* sample file for subscribing point cloud
* add /usr/bin/env roseus
* fix for bvh does not have 'site
* fix: update for publishing /ObjectDetection in tabletop-object-detector
* temporary update
* temporary update
* update for fuerte
* fix: frame_id of openni_tracker
* fix: delete old include
* fix: xml
* fix: kinect.launch for fuerte
* fix for non-jsk users
* fix for non-jsk users
* change joy::Joy -> std_msgs::Joy
* copy tabletop_segmentation.launch from tabletop_object_detector to fix zfilter_max
* use lifetime for marker
* set 900 as default table surface, add debug message, check ROS_MASTER_URI to use req.table
* add the code to manually set the table plane
* outout launchdoc-generator to build directry to avoid svn confrict
* force add 'site to the link-list
* add *transform-table* flag for transforming bounding box's coordinates on table
* add loop-hook argument which is a function to be called inside do-until-key loop
* rename openni_swipe.l -> openni-swipe.l  openni_pointer.l -> openni-pointer.l
* update openni-swipe.l
* update openni-pointer.l, change led light due to server status
* add openni_pointer.l openni_swipe.l
* update description of tabletop_detector.launch
* update location of facedetect database
* update for detecting 1000yen
* remove kinet.launch and tabletop-object-detector.l and write the documents
* execute .l file in tabletop_object_detector.launch
* add tabletop-object-detector
* update fix-joint-order,fix-joint-angle,bvh-offset-rotate for kinect-bvh-robot-model
* write bvh file if :fname is defined
* add object 4x4 with 70mm x 70mm
* fix typo about aques_talk's pronunciation
* rename j_robotsound -> robotsound_jp
* changed topic name for aques_talk speech node
* update documents
* send transform at time marker is captured
* add depends to ar_pose
* add description of euslisp client example
* add ar-pose.launch and ar-pose.l
* add kinect tracker example
* add smple to use :args2 for SoundRequest::*say*
* fix aques-talk.launch for r2145 of aques_talk/text2wave
* added markerarray samples
* set default blurry mode to to false
* add how to launch example
* add blurry mode sample
* docs
* rosdoc yaml changes
* doc updates
* add conf.py index.rst
* fix revert-if-error -> revert-if-fail
* rename node name for vision-action-example{1,2,3}.l
* add comment to CMakeLists.txt to run rosdoc when you make roseus_tutorials
* fix for new message compile rule
* fix image_view2::ImageMarker:: -> image_view2::ImageMarker2
* fix image_view2::ImageMarker:: -> image_view2::ImageMarker2
* fix typo image_view2::ImageMarker::*POLYGON* -> image_view2::ImageMarker2::*POLYGON*
* minor doc stuff
* more autodoc stuff
* auto-generation of roslaunch docs
* add launch/images/
* slow down for note pc
* slow down for note pc
* slow down for note pc
* update constant message definition to PACKAGE::FILE::VARIABLE style
* add vision-action-example
* fix debug message
* fix debug message
* use load-ros-manifest, instead of roseus-add-msgs for sample program
* remove imgae-proc.launch, image proc is executed in usb-camera.launch
* use uvc_cmaera instaed of usb_cam
* fix for new defconstant msg compile rule
* fix for new aques_talk
* add move verbose
* use imagesurf instead of imagesift
* change frame_id from camera to usb_cam
* update color info
* update tutorials
* set color-skin.txt in cmvision.launch
* add move verbose
* use uvc_camera, instad of uvc_cam
* update for new roseus message defconstant with **
* remove image data and download from www.boj.or.jp
* remove image data and download from www.boj.or.jp
* change template image
* remove jsk_mep dependency
* add kinect.launch
* add window_name to launch files
* add <mihon> mark in one-thousand yen bill image
* add image_view to template-match samples
* updating for roseus_tutorial with diamondback
* add executable property to roseus_tutorials/src/*.l
* add one thousand yen bill image, do not print this
* update for diamondback roslib -> std_msgs
* add point-pose-extraction.l and launch file by ishida
* change image_marker advertise buffer from 1->10
* fix screenrectangle remap
* add camshiftdemo
* remove template-track.l from launch file
* remove jsk_mep_converter is is obsoleted
* change package name jsk_mep_converter -> jsk_perception
* fix : moving files from jsk-ros-pkg-unrelased to jsk-ros-pkg corrupt some files
* add cmvision,saliency-track,image-ivew,image-proc,usb-camera,checkerboard-pose launch file for tutorial
* update publish-marker, publish cube and sphere marker
* fix for empty tag, insert slash before close bracket
* update publish-marker.launch to run rviz and add publish-marker.vcg for rviz display_config
* change to use roseus, whcih automatically load roseus.l eustf.l actionlib.l
* add publish_marker example by t-ito
* add roseus_tutorials
* Contributors: Haseru Chen, Rosen Diankov, Shunsuke Nozawa, Manabu Saito, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Eisoku kuroiwa, Atushi Tsuda, Ryohei Ueda, Tukasa Ito, Youhei Kakiuchi
