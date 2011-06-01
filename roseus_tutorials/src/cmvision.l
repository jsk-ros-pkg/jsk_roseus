#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "cmvision")
;;;
;;;
(load "package://pr2eus/pr2.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(objects (list *pr2*))

(ros::roseus "cmvisoin-client")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start cmvision.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./cmvision.l"))

(defun cmvision-cb (blobs)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) p)
    (dotimes (i (send blobs :blob_count))
      (let ((b (elt (send blobs :blobs) i)))
        (ros::ros-info "~3d:area:~6d, x ~3d, y~3d" i (send b :area) (send b :x)
 (send b :y))
        (push (instance geometry_msgs::Point :init :x (send b :x) :y (send b :y)) p)
        ))
    (send mrk :type image_view2::ImageMarker2::*POINTS*)
    (send mrk :points p)
    (ros::publish "image_marker" mrk)

    (when (> (send blobs :blob_count) 0)
      (let ((b (car (send blobs :blobs))))
        ;;
        (send *pr2* :head :neck-y :joint-angle (if (> (send b :x) 320) +10 -10) :relative t)
        (send mrk :type image_view2::ImageMarker2::*POLYGON*)
        (send mrk :points (list
                           (instance geometry_msgs::Point :init
                                     :x (send b :left) :y (send b :top))
                           (instance geometry_msgs::Point :init
                                     :x (send b :right) :y (send b :top))
                           (instance geometry_msgs::Point :init
                                     :x (send b :right) :y (send b :bottom))
                           (instance geometry_msgs::Point :init
                                     :x (send b :left) :y (send b :bottom))))))
    (ros::publish "image_marker" mrk)
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "blobs" cmvision::Blobs #'cmvision-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  (send *irtviewer* :draw-objects)
  )
