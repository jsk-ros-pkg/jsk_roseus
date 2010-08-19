#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;

;;(setq ros::*compile-message* t) ;; comment out if you want to compile msg and srv
(load-ros-manifest "roseus")
;;;

;;;
;;;
(roseus "talker")
(ros::advertise "chatter" std_msgs::string 1)
(ros::rate 100)
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "chatter" msg)
  (ros::sleep)
  )
(roseus "shutdown")
(exit)

