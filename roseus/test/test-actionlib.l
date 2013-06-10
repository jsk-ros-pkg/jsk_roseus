#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib_tutorials")

(ros::roseus "test_actionlib")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::service-call "~set_logger_level"
                   (instance roscpp::SetLoggerLevelRequest
                             :init :logger "ros" :level "DEBUG"))
;;;
;;;

(deftest wait-for-result-twice
  (let (goal result)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" actionlib_tutorials::FibonacciAction)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order 10)
    (send *c* :send-goal goal)
    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (assert (setq result (send *c* :wait-for-result)) "wait-for-result -> ~A" result)
    (setq result (send (send *c* :get-result) :sequence))
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) result)
    (assert (= (elt result (1- (length result))) 55) "(fib 10) -> ~A" result)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (assert (setq result (send *c* :wait-for-result)) "wait-for-result -> ~A" result) ;; note that wait-for-result returns t
    (setq result (send (send *c* :get-result) :sequence))
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) result)
    (assert (= (elt result (1- (length result))) 55) "(fib 10) -> ~A" result)
    ))


(deftest wait-for-result-without-send-goal
  (let ()
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" actionlib_tutorials::FibonacciAction)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (assert (send *c* :wait-for-result)) ;; note that :wait-for-results returns t even if no goal has need sent
    ))

(defun fibonacci (n)
  (if (< n 2)
      n
    (+ (fibonacci (- n 1)) (fibonacci (- n 2)))))

(defun fibonacci-client (&optional (order 10))
  (let (goal)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" actionlib_tutorials::FibonacciAction)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order order)
    (send *c* :send-goal goal)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send *c* :get-result) :sequence))
    (send (send *c* :get-result) :sequence)))


(deftest fibonacci-client-10
  (let (ans fib)
    (dotimes (count 10)
      (setq order (+ 16 (random 20)))
      (setq ans (fibonacci order))
      (warning-message 3 ";; test-fibonacci ~A ~A/~A~%" order count 10)
      (setq fib (fibonacci-client order))
      (if (> (length fib) 0)
          (assert (= (elt fib (1- (length fib))) ans) "(fib ~A) -> ~A/~A" order ans fib))
    )))

(deftest fibonacci-client-300
  (let (ans fib)
    (dotimes (count 300)
      (setq order (+ 16 (random 16)))
      (setq ans (fibonacci order))
      (warning-message 3 ";; test-fibonacci ~A ~A/~A~%" order count 300)
      (setq fib (fibonacci-client order))
      (if (> (length fib) 0)
          (assert (= (elt fib (1- (length fib))) ans) "(fib ~A) -> ~A/~A" order ans fib))
    )))

(run-all-tests)
(exit)

