#!/usr/bin/env runeus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-srvs "rospy_tutorials")
;;;
;;;
(roseus "add_two_ints_client")
(ros::wait-for-service "add_two_ints")
(dotimes (i 100)
  (setq req (instance rospy_tutorials::AddTwoIntsRequest :init))
  (send req :a (random 10))
  (send req :b (random 20))
  (setq res (ros::service-call "add_two_ints" req))
  (format t "~d + ~d = ~d~%" (send req :a) (send req :b) (send res :sum))
  (unix:sleep 1))




