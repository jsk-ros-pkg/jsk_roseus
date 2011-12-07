#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "ar_pose")

(ros::roseus "ar_pose_clinet")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun ar-pose-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) c)
    (dolist (marker (send msg :markers))
      (setq c (ros::tf-pose->coords (send marker :pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list "/ar_marker_frame"))
      (send *tfb* :send-transform
	    c (send marker :header :frame_id) "/ar_marker_frame"
	    (send marker :header :stamp))
      (ros::ros-info "~A" c)
      (ros::publish "image_marker" mrk)
      )))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "ar_pose_marker" ar_pose::ARMarkers #'ar-pose-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (x::window-main-one)
  (ros::sleep)
  )

