#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "jsk_perception")
;;;
;;;
(ros::roseus "camshiftdemo")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start camshiftdemo.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./camshiftdemo.l"))

;; this argument msg reperesents 2d-pose
(defun camshift-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
        width height angle)
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :position (instance geometry_msgs::Point :init
                                  :x (send msg :rect :x)
                                  :y (send msg :rect :y)))
    (send mrk :scale 20)
    (ros::publish "image_marker" mrk)

    (send mrk :type image_view2::ImageMarker2::*LINE_LIST*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)

    (setq angle (deg2rad (send msg :rect :angle)))
    (setq width  (/ (send msg :rect :width) 2)
          height (/ (send msg :rect :height) 2))
    (format t "camshift-cb w:~A h:~A deg:~A~%" width height (rad2deg angle))
    (send mrk :points
          (list
           (instance geometry_msgs::Point :init
                     :x (+ (send msg :rect :x) (* width (cos angle)))
                     :y (+ (send msg :rect :y) (* width (sin angle))))
           (instance geometry_msgs::Point :init
                     :x (- (send msg :rect :x) (* width (cos angle)))
                     :y (- (send msg :rect :y) (* width (sin angle))))
           (instance geometry_msgs::Point :init
                     :x (+ (send msg :rect :x) (* height (cos (+ angle pi/2))))
                     :y (+ (send msg :rect :y) (* height (sin (+ angle pi/2)))))
           (instance geometry_msgs::Point :init
                     :x (- (send msg :rect :x) (* height (cos (+ angle pi/2))))
                     :y (- (send msg :rect :y) (* height (sin (+ angle pi/2)))))))
    (ros::publish "image_marker" mrk)
    
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 10)
(ros::subscribe "camshiftdemo/result" jsk_perception::RotatedRectStamped #'camshift-cb)


(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )
