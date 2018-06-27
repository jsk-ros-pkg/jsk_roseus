#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "test_name_sub")
(ros::subscribe "test_name" roseus::TestName #'(lambda (msg) (ros::ros-info "sub ~A" msg)))
(ros::spin)

