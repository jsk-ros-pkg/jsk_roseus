#!/usr/bin/env roseus

(ros::ros-info "~~test => ~A" (ros::get-param "~test"))
(ros::ros-info "test => ~A" (ros::get-param "test"))
(ros::ros-info "/test => ~A" (ros::get-param "/test"))

(ros::roseus "param_test")
(ros::ros-info "~~test => ~A" (ros::get-param "~test"))
(ros::ros-info "test => ~A" (ros::get-param "test"))
(ros::ros-info "/test => ~A" (ros::get-param "/test"))

