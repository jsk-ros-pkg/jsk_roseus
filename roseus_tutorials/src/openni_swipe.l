#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus_tutorials")

(ros::roseus "openni_swipe")

(setq *status* 1) ;; IN_SESSION NOT_IN_SESSION QUICK_REFOCUS
(setq *gesture* nil)
(ros::advertise "image_marker" image_view2::ImageMarker2 10)
(ros::advertise "/led_option" std_msgs::UInt16 10)

(ros::subscribe "/swipe" std_msgs::String
		#'(lambda (msg)
		    (let ((mrk (instance image_view2::ImageMarker2 :init)))
		      (setq *gesture* (send msg :data))
		      (ros::ros-info "~A" *gesture*)
		      (cond ((string= *gesture* "StandBy"))
			    (t
			     (send mrk :type image_view2::ImageMarker2::*Text*)
			     (send mrk :position (instance geometry_msgs::Point :init :x 70 :y 40))
			     (send mrk :text *gesture*)
			     (send mrk :lifetime (ros::time 1))
			     (ros::publish "image_marker" mrk)))
		      )))
(ros::subscribe "/swipe/status" std_msgs::UInt8
		#'(lambda (msg) (setq *status* (send msg :data))))

;; main
(ros::rate 30)
(setq *led_option* (instance std_msgs::UInt16 :init :data 2))
(ros::publish "/led_option" *led_option*)
(do-until-key
 (let ((led_option (case *status*
		     (0 1)   ;; IN_SESSION
		     (1 2)   ;; NOT_IN_SESSION
		     (2 3))));; QUICK_REFOCUS
   (if (string= *gesture* "StandBy") (setq led_option 5))
   (when (/= (send *led_option* :data) led_option)
     (send *led_option* :data led_option)
     (ros::publish "/led_option" *led_option*))
   ;; off, gree, red, yellow, blink yellow, blink green, blink red
  (ros::spin-once)
  (ros::sleep)
  ))

