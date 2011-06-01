#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
;;;
;;;
(ros::roseus "template_tracking_marker")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start template-track.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./template-track.l"))

;; this argument msg reperesents 2d-pose
(defun tracking-cb (trans-msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
	(pt (instance geometry_msgs::Point :init)))
    (send pt :x (send trans-msg :transform :translation :x))
    (send pt :y (send trans-msg :transform :translation :y))

    (send mrk :ns (send trans-msg :child_frame_id))
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :position pt)
    (send mrk :scale 20)

    (ros::publish "image_marker" mrk)
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "result" geometry_msgs::TransformStamped #'tracking-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
