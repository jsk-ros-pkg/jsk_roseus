#!/usr/bin/env runeus

(load "roseus.l")

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "cmvision")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
;;;
;;;
(roseus "cmvisoin-client")

(defun cmvision-cb (blobs)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) p)
    (dotimes (i (send blobs :blob_count))
      (let ((b (elt (send blobs :blobs) i)))
        (ros::ros-info "~3d:area:~6d, x ~3d, y~3d" i (send b :area) (send b :x)
 (send b :y))
        (push (instance geometry_msgs::Point :init :x (send b :x) :y (send b :y)) p)
        ))
    (send mrk :type image_view2::POINTS)
    (send mrk :points p)
    (ros::publish "image_marker" mrk)

    (when (> (send blobs :blob_count) 0)
      (let ((b (car (send blobs :blobs))))
        (send mrk :type image_view2::POLYGON)
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
  )

