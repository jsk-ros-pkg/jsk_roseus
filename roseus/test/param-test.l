#!/usr/bin/env roseus

(ros::roseus "param_test")
(unix:sleep 1)
(ros::ros-info "~~test => ~A" (ros::get-param "~test"))
(ros::ros-info "test => ~A" (ros::get-param "test"))
(ros::ros-info "/test => ~A" (ros::get-param "/test"))
(ros::spin)
