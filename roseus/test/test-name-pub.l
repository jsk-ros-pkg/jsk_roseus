#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "test_name_pub")
(ros::advertise "test_name" roseus::TestName)
(ros::rate 1)
(setq i 0)
(while (ros::ok)
  (setq ss (instance roseus::StringStamped :init :data (format nil "test ~A" (incf i))))
  (setq msg (instance roseus::TestName :init :name ss))
  (ros::ros-info "pub ~A" msg)
  (ros::publish "test_name" msg)
  (ros::sleep)
  (ros::spin-once))

