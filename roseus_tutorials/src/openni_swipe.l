#!/usr/bin/env roseus

(ros::load-ros-manifest "openni_poite")

(ros::roseus "openni_pointer")

(ros::subscribe "/hand_position" geometry_msgs::PointStamped
		#'(lambda (msg)
		    (print (send msg :point))))

;; main
(ros::rate 10)
(do-until-key
  (ros::spin-once)
  (ros::sleep)
  )

