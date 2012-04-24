#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus_tutorials")

(ros::roseus "openni_swipe")

(setq *status* 1) ;; IN_SESSION NOT_IN_SESSION QUICK_REFOCUS
(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::advertise "/led_option" std_msgs::UInt16 1)
(ros::subscribe "/hand_position" geometry_msgs::PointStamped
		#'(lambda (msg)
		    (let ((mrk (instance image_view2::ImageMarker2 :init)))
		      (when (= *status* 0)
			(ros::ros-info "hand position ~7,3f ~7,3f" (send msg :point :x) (send msg :point :y))
			(send mrk :type image_view2::ImageMarker2::*Points*)
			(send mrk :header (send msg :header))
			(send mrk :points (list (send msg :point)))
			(send mrk :lifetime (ros::time 1))
			(ros::publish "image_marker" mrk)
			))))
(ros::subscribe "/hand_position/status" std_msgs::UInt8
		#'(lambda (msg) (setq *status* (send msg :data))))

;; main
(ros::rate 30)
(setq *led_option* (instance std_msgs::UInt16 :init))
(do-until-key
 (let ((led_option (case *status*
		     (0 5)   ;; IN_SESSION
		     (1 2)   ;; NOT_IN_SESSION
		     (2 3))));; QUICK_REFOCUS
   (when (/= (send *led_option* :data) led_option)
     (send *led_option* :data led_option)
     (ros::publish "/led_option" *led_option*))
   ;; off, gree, red, yellow, blink yellow, blink green, blink red
   (ros::spin-once)
   (ros::sleep)
   ))

