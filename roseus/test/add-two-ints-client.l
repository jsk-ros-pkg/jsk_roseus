#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "roseus")

;;;
;;;
(ros::roseus "add_two_ints_client")
(ros::wait-for-service "add_two_ints")
(dotimes (i 100)
  (setq req (instance roseus::AddTwoIntsRequest :init))
  (send req :a (random 10))
  (send req :b (random 20))
  (setq before (ros::time-now))
  (case (mod i 3)
    (0 (setq res (ros::service-call "add_two_ints" req t)))
    (1 (setq res (ros::service-call "add_two_ints" req nil)))
    (2 (setq res (ros::service-call "add_two_ints" req))))
  (setq after (ros::time-now))
  (format t "~d + ~d = ~d~ (~A sec)~%" (send req :a) (send req :b) (send res :sum) (send (ros::time- after before) :to-sec))
  (unix:sleep 1))




