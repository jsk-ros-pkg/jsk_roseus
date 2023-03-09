#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "roseus")

;;;
(defun add-two-floats (req)
  (let ((m (send req :response)))
    (ros::ros-info "Returning [~d + ~d = ~d]"
            (send req :a) (send req :b)
            (+ (send req :a) (send req :b)))
    (send m :sum  (+ (send req :a) (send req :b)))
    m))
;;;
;;;
(ros::roseus "add_two_floats_server")
(ros::advertise-service "add_two_floats" roseus::AddTwoFloats #'add-two-floats)
(do-until-key
 (ros::spin-once))
