#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
;;;
;;;

(ros::roseus "sliency-client")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start saliency-tracking.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./saliency-tracking.l"))

(defun saliency-cb (poi)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
        (x (send poi :x)) (y (send poi :y)))
    (print (list x y))
    (ros::ros-info "~A" poi)

    (send mrk :type image_view2::ImageMarker2::*POINTS*)
    (send mrk :points (list poi))
    (ros::publish "image_marker" mrk)

    (send mrk :type image_view2::ImageMarker2::*POLYGON*)
    (send mrk :points (list
                       (instance geometry_msgs::Point :init :x (- x 8) :y (- y 8))
                       (instance geometry_msgs::Point :init :x (- x 8) :y (+ y 8))
                       (instance geometry_msgs::Point :init :x (+ x 8) :y (+ y 8))
                       (instance geometry_msgs::Point :init :x (+ x 8) :y (- y 8))))
    (ros::publish "image_marker" mrk)
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "saliency_poi" geometry_msgs::Point #'saliency-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
