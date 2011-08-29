#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_server.py
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib_tutorials")
;;;

;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "fibonacci")
(defun fibonacci-execute-cb (server goal)
  (let ((order (send goal :goal :order))
        (sequence (list 1 0)) msg)
    (dotimes (i (1- order))
      (push (+ (car sequence) (cadr sequence)) sequence))
    (setq msg (send server :result :sequence (nreverse sequence)))
    (send server :set-succeeded msg)
    ))

(setq s (instance ros::simple-action-server :init
                  "/fibonacci" actionlib_tutorials::FibonacciAction
                  :execute-cb #'fibonacci-execute-cb))

(ros::rate 500)
(do-until-key
 (send s :worker)
 (ros::spin-once)
 (ros::sleep))




