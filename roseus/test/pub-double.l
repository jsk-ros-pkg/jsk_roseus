#!/usr/bin/env roseus
;;;
;;;
(ros::load-ros-manifest "roseus")

(ros::roseus "pub_double")
(ros::advertise "double" std_msgs::float64 1)
(ros::rate 10)
(setq f 123456.654321)
(while (ros::ok)
  (setq msg (instance std_msgs::float64 :init))
  (send msg :data f)
  (warning-message 3 "publish msg [~8,8f]~%" (send msg :data))
  (ros::publish "double" msg)
  (ros::sleep)
  (setq f (* f 2))
  )
(ros::roseus "shutdown")
(exit)

