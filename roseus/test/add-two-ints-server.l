#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-srvs "rospy_tutorials")
;;;

(defun add-two-ints (req)
  (let ((m (send req :response)))
    (format *error-output* "Returning [~d + ~d = ~d]~%"
            (send req :a) (send req :b)
            (+ (send req :a) (send req :b)))
    (send m :sum  (+ (send req :a) (send req :b)))
    m))
;;;
;;;
(ros::roseus "add_two_ints_server")
(ros::advertise-service "add_two_ints" rospy_tutorials::AddTwoInts #'add-two-ints)
(do-until-key
 (ros::spin-once))




