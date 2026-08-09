#!/usr/bin/env roseus
;;;
;;; euslisp version of https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29 (ROS 1)
;;;
(ros::load-ros-manifest "example_interfaces")

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "fibonacci")

(defun fibonacci-execute-cb (server goal)
  (let ((order (send goal :goal :order))
        (sequence (list 1 0)) msg)
    (dotimes (i (1- order))
      (push (+ (car sequence) (cadr sequence)) sequence)
      (setq msg (send server :feedback :sequence (reverse sequence)))
      (send server :publish-feedback msg)
      )
    (setq msg (send server :result :sequence (nreverse sequence)))
    (send server :set-succeeded msg)
    ))

(setq s (instance ros::simple-action-server :init
                  "fibonacci" example_interfaces::Fibonacci
                  :execute-cb #'fibonacci-execute-cb))

(ros::rate 500)
(do-until-key
    (send s :worker)
  (ros::sleep))
