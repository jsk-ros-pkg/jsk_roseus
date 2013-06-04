#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib_tutorials")
;;;
(when (member "--gtest_output=xml:" lisp::*eustop-argument* :test #'substringp)  ;; for test-fibonacci.launch
  (require :unittest "lib/llib/unittest.l")
  (init-unit-test))

;;;
;;;
(ros::roseus "fibonacci_client")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(defun fibonacci (n)
  (if (< n 2)
      n
    (+ (fibonacci (- n 1)) (fibonacci (- n 2)))))

(defun fibonacci-client (&optional (order 10))
  (let (goal)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" actionlib_tutorials::FibonacciAction)))
    (warning-message 1 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order order)
    (send *c* :send-goal goal)

    (warning-message 1 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 1 ";; ~A result -> ~A~%" (unix::getpid) (send (send *c* :get-result) :sequence))
    (send (send *c* :get-result) :sequence)))

(unless (boundp '*unit-test*)
  (let ((count 0) ans fib)  ;; for test-fibonaci.launch
    (setq ans (fibonacci 10))
    (do-until-key
     (warning-message 2 "test-fibonacci ~A/~A~%" count 1000)
     (incf count)
     (when (> count 1000)
       (exit 0))
     (setq fib (fibonacci-client 10))
     (assert (= (elt fib (1- (length fib))) ans) "(fib 10) -> ~A" fib))
    (exit 0)
    ))

(deftest fibonacci-client-300
  (let (ans fib)
    (dotimes (count 300)
      (setq order (+ 16 (random 16)))
      (setq ans (fibonacci order))
      (warning-message 2 "test-fibonacci ~A ~A/~A~%" order count 300)
      (setq fib (fibonacci-client order))
      (assert (= (elt fib (1- (length fib))) ans) "(fib ~A) -> ~A/~A" order ans fib)
    )))

(run-all-tests)
(exit)

