#!/usr/bin/env roseus
;;;
;;; ROS 2 action test (equivalent of test-actionlib.l for ROS 1)
;;; Uses roseus::Fibonacci action directly.
;;;
(ros::load-ros-manifest "roseus")
(require :actionlib2 "actionlib2.l")

(ros::roseus "test_actionlib_ros2")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;;;
;;; Test 1: Send goal, wait for result, get result, then wait again (cached)
;;;
(deftest wait-for-result-twice
  (let (goal result)
    (setq *c* (instance ros::simple-action-client :init
                        "fibonacci" roseus::Fibonacci))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance roseus::Fibonacci_SendGoalRequest :init))
    (send goal :goal :order 10)
    (send *c* :send-goal goal)
    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (assert (setq result (send *c* :wait-for-result)) "wait-for-result -> ~A" result)
    (setq result (send (send *c* :get-result) :sequence))
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) result)
    (assert (= (elt result (1- (length result))) 55) "(fib 10) -> ~A" result)

    ;; Second wait should return cached result
    (warning-message 2 ";; ~A wait-for-result (cached)~%" (unix::getpid))
    (assert (setq result (send *c* :wait-for-result)) "wait-for-result -> ~A" result)
    (setq result (send (send *c* :get-result) :sequence))
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) result)
    (assert (= (elt result (1- (length result))) 55) "(fib 10) -> ~A" result)
    ))

;;;
;;; Test 2: Wait for result without sending a goal (should return nil)
;;;
(deftest wait-for-result-without-send-goal
  (let ()
    (warning-message 2 ";; ~A wait-for-result (no goal)~%" (unix::getpid))
    ;; Reuse *c* from previous test - it already has a cached result but no new goal
    ;; Create a fresh client state by sending no goal
    (setq *c* (instance ros::simple-action-client :init
                        "fibonacci" roseus::Fibonacci))
    (send *c* :wait-for-server)
    (assert (null (send *c* :wait-for-result)) "wait-for-result should be nil when no goal sent")
    ))

;;;
;;; Test 3: Send goal using Goal directly (not SendGoalRequest)
;;;
(deftest send-goal-directly
  (let (goal result)
    (warning-message 2 ";; ~A send-goal-directly~%" (unix::getpid))
    (setq goal (instance roseus::Fibonacci_Goal :init))
    (send goal :order 7)
    (send *c* :send-goal goal)
    (assert (send *c* :wait-for-result) "wait-for-result")
    (setq result (send (send *c* :get-result) :sequence))
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) result)
    ;; fib(7) = 13, sequence = [0, 1, 1, 2, 3, 5, 8, 13]
    (assert (= (elt result (1- (length result))) 13) "(fib 7) -> ~A" result)
    (assert (= (length result) 8) "length(fib(7)) = 8, got ~A" (length result))
    ))

;;;
;;; Test 4: Multiple goals through same client with correctness check
;;;
(defun fibonacci (n)
  (if (< n 2)
      n
    (+ (fibonacci (- n 1)) (fibonacci (- n 2)))))

(deftest fibonacci-client-10
  (let (ans fib goal)
    (dotimes (count 10)
      (setq order (+ 5 (random 15)))
      (setq ans (fibonacci order))
      (warning-message 3 ";; test-fibonacci ~A ~A/~A~%" order count 10)
      (setq goal (instance roseus::Fibonacci_Goal :init))
      (send goal :order order)
      (send *c* :send-goal goal)
      (send *c* :wait-for-result)
      (setq fib (send (send *c* :get-result) :sequence))
      (warning-message 2 ";; ~A result ~A: last=~A expected=~A~%"
                       (unix::getpid) count
                       (elt fib (1- (length fib))) ans)
      (if (> (length fib) 0)
          (assert (= (elt fib (1- (length fib))) ans) "(fib ~A) -> ~A/~A" order ans fib))
      )))

(run-all-tests)
(exit)
