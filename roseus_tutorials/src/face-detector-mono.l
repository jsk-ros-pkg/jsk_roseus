#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "opencv_apps")
;;;
;;;
(ros::roseus "facedetect-client")

(defun facedetect-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) ret)
    (warning-message 1 "facedetect callback~%")
    (dolist (f (send msg :faces))
      (format t "cx:~3d, cy:~3d~%" (send f :face :x) (send f :face :y)))

    (when (send msg :faces)
      (let* ((f (car (send msg :faces)))
             (cx (send f :face :x)) (cy (send f :face :y))
             (w  (send f :face :width))  (w/2 (/ w 2))
             (h  (send f :face :height)) (h/2 (/ h 2)))
        (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)

        (send mrk :type image_view2::ImageMarker2::*POLYGON*)
        (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :R 0 :G 1 :B 0)))
        (send mrk :lifetime (ros::time 5))
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
(ros::subscribe "face_detection/faces" opencv_apps::FaceArrayStamped #'facedetect-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
