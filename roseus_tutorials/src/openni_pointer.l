#!/usr/bin/env roseus

(ros::load-ros-manifest "openni_swipe")

(ros::roseus "openni_swipe")

(ros::subscribe "/swipe" std_msgs::String
		#'(lambda (msg)
		    (print (send msg :data))))

;; main
(ros::rate 10)
(do-until-key
  (ros::spin-once)
  (ros::sleep)
  )

