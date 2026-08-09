#!/usr/bin/env roseus
;;;
;;; euslisp version of fibonacci_client.l (ROS 2)
;;; cf. roseus/test/fibonacci-client.l (ROS 1)
;;;
(ros::load-ros-manifest "roseus")
(require :actionlib2 "actionlib2.l")

(ros::roseus "fibonacci_client")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;;;
;;;
(defun fibonacci (n)
  (if (< n 2)
      n
    (+ (fibonacci (- n 1)) (fibonacci (- n 2)))))

(defun feedback-cb
  (msg)
  (format t "feedback sequence: ~a~%" (send msg :sequence))
  )

(defun fibonacci-client (&optional (order 10))
  (let (goal)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" roseus::Fibonacci)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance roseus::Fibonacci_Goal :init :order order))
    (send *c* :send-goal goal :feedback-cb #'feedback-cb)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send *c* :get-result) :sequence))
    (send (send *c* :get-result) :sequence)))

(let ((count 0) ans fib)
  (setq ans (fibonacci 10))
  (do-until-key
   (warning-message 4 ";; test-fibonacci ~A/~A~%" count 1000)
   (incf count)
   (when (> count 2)
     (exit 0))
   (setq fib (fibonacci-client 10))
   (if (> (length fib) 0)
       (assert (= (elt fib (1- (length fib))) ans) "(fib 10) -> ~A" fib))
  (exit 0)
  ))
