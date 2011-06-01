#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "face_detector_mono")
;;;
;;;
(ros::roseus "facedetect-client")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start facedetect-client.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./facedetect-client.l"))

(defun facedetect-cb (faces)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) ret)
    (warning-message 1 "facedetect callback~%")
    (dolist (f (send faces :rects))
      (format t "cx:~3d, cy:~3d~%" (send f :x) (send f :y)))

    (when (send faces :rects)
      (let* ((f (car (send faces :rects)))
             (cx (send f :x)) (cy (send f :y))
             (w  (send f :width))  (w/2 (/ w 2))
             (h  (send f :height)) (h/2 (/ h 2)))
        (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)

        (send mrk :type image_view2::ImageMarker2::*POLYGON*)
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
(ros::subscribe "face_detector_mono/faces" face_detector_mono::RectArray #'facedetect-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
