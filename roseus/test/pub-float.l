#!/usr/bin/env roseus
;;;
;;;
(ros::load-ros-manifest "roseus")

(ros::roseus "pub_float")
(ros::advertise "float" std_msgs::float32 1)
(ros::rate 10)
(setq f 123.321)
(while (ros::ok)
  (setq msg (instance std_msgs::float32 :init))
  (send msg :data f)
  (warning-message 3 "publish msg [~8,8f]~%" (send msg :data))
  (ros::publish "float" msg)
  (ros::sleep)
  (setq f (* f 2))
  )
(ros::roseus "shutdown")
(exit)

