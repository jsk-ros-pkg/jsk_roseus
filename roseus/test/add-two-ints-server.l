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

(defun add-two-ints (req)
  (setq m (send req :response))
  (format t "Returning [~d + ~d = ~d]~%" 
          (send req :a) (send req :b)
          (+ (send req :a) (send req :b)))
  (send m :sum  (+ (send req :a) (send req :b)))
  m)
;;;
;;;
(roseus "add_two_ints_server")
(ros::advertise-service "add_two_ints" rospy_tutorials::AddTwoInts #'add-two-ints)
(ros::spin)




