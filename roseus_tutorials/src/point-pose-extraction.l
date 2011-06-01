#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "jsk_perception")


(ros::roseus "point-pose-ectraction")

(defun detection-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
	(pt (instance geometry_msgs::Point :init)))
    (send pt :x (* 1000 (send (car (send msg :objects)) :pose :position :x)))
    (send pt :y (* 1000 (send (car (send msg :objects)) :pose :position :y)))
    (print (list (send pt :x) (send pt :y)))

    (send mrk :ns (send msg :header :frame_id))
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :position pt)
    (send mrk :scale 20)

    (ros::publish "image_marker" mrk)
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection
		#'detection-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
