#!/usr/bin/env runeus

(load "roseus.l")

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
;;;
;;;

(roseus "sliency-client")

(defun saliency-cb (poi)
  (setq mrk (instance image_view2::ImageMarker2 :init))
  (print (list (send poi :x) (send poi :y)))
  (send mrk :type image_view2::POINTS)
  (send mrk :points (list poi))
  (ros::ros-info "~A" poi)
  (ros::publish "image_marker" mrk)
  )

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "saliency_poi" geometry_msgs::Point #'saliency-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
