#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "test_name_pub")
(ros::advertise "test_name" roseus::TestName)
(ros::rate 1)
(setq i 0)

;; Test for name clashing in ROS messages
;; `name' tests for name clashing in slot names
;; `string' tests for name clashing in message names

(while (ros::ok)
  (let* ((str (format nil "test ~A" (incf i)))
         (s (instance roseus::String :init :data str))
         (ss (instance roseus::StringStamped :init :data str))
         (msg (instance roseus::TestName :init :name ss :string s)))
    (ros::ros-info "pub ~A" msg)
    (ros::publish "test_name" msg)
    (ros::sleep)
    (ros::spin-once)))

