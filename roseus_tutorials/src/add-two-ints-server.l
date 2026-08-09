#!/usr/bin/env roseus
;;;
;;; euslisp version of https://github.com/ros2/demos/blob/rolling/demo_nodes_py/demo_nodes_py/services/add_two_ints_server.py
;;;
(ros::load-ros-manifest "example_interfaces")

(defun add-two-ints (req)
  (let ((m (send req :response)))
    (format *error-output* "Returning [~d + ~d = ~d]~%"
            (send req :a) (send req :b)
            (+ (send req :a) (send req :b)))
    (send m :sum  (+ (send req :a) (send req :b)))
    m))

(ros::roseus "add_two_ints_server")
(ros::advertise-service "add_two_ints" example_interfaces::AddTwoInts #'add-two-ints)
(do-until-key
 (ros::spin-once))
