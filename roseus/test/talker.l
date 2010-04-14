#!/usr/bin/env runeus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
;;;

;;;
;;;
(roseus "talker")
(ros::advertise "chatter" std_msgs::string 1)
(ros::rate 10)
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (ros::time-now)))
  ;;(ros::ros-info (send msg :data))
  (ros::ros-info msg)
  (ros::publish "chatter" msg)
  (ros::spin-once)
  (ros::sleep)
  )
(roseus "shutdown")
(exit)

