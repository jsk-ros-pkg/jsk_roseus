#!/usr/bin/env roseus
;;;
;;; ROS 2 version of test-add-two-ints-server.l
;;; Excluded: connection-header assertion (not available in ROS 2 DDS)

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

;; Number of calls add-two-ints-client-ros2.l makes.
(defvar *expected-calls* 20)
(defvar *num-served* 0)

(defun add-two-ints (req)
  (let ((m (send req :response)))
    (warning-message 2 "Returning [~d + ~d = ~d]~%"
                     (send req :a) (send req :b)
                     (+ (send req :a) (send req :b)))
    (send m :sum  (+ (send req :a) (send req :b)))
    (incf *num-served*)
    m))

(deftest test-add-two-ints-server
  (ros::roseus "add_two_ints_server")
  (ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints)

  (ros::rate 100)
  (let ((max-iter 1500) (i 0))
    (while (and (< i max-iter) (< *num-served* *expected-calls*))
      (ros::spin-once)
      (ros::sleep)
      (incf i)))
  (warning-message 2 "served ~A/~A requests~%" *num-served* *expected-calls*)
  (assert (>= *num-served* *expected-calls*)
          (format nil "served only ~A of ~A requests"
                  *num-served* *expected-calls*))
  )

(ros::roseus "add_two_ints_server_init")
(run-all-tests)

(exit)
