#!/usr/bin/env roseus

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "recognition_pipeline")
;;;
;;;
(roseus "facedetect-client")

(defun facedetect-cb (faces)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) ret)
    (dolist (f (send faces :rects))
      (print (list (send f :x) (send f :y))))

    (when (send faces :rects)
      (let* ((f (car (send faces :rects)))
             (cx (send f :x)) (cy (send f :y))
             (w  (send f :width))  (w/2 (/ w 2))
             (h  (send f :height)) (h/2 (/ h 2)))
        (print (list cx cy w h))

        (send mrk :type image_view2::POLYGON)
        (send mrk :points (list
                           (instance geometry_msgs::Point :init
                                     :x (- cx w/2) :y (- cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (- cx w/2) :y (+ cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (+ cx w/2) :y (+ cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (+ cx w/2) :y (- cy h/2))))
        (ros::publish "image_marker" mrk)))))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "/facedetect/faces" recognition_pipeline::RectArray #'facedetect-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
