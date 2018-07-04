#!/usr/bin/env roseus
;;;
;;;

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

(defun add-two-ints (req)
  (let ((m (send req :response)))
    (warning-message 2 "Returning [~d + ~d = ~d] (~A)~%"
                     (send req :a) (send req :b)
                     (+ (send req :a) (send req :b))
                     (send req :connection-header))
    (assert (send req :connection-header) "connection-header is not found")
    (send m :sum  (+ (send req :a) (send req :b)))
    m))

(deftest test-add-two-ints-server
  (ros::roseus "add_two_ints_server")
  (ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints)

  (ros::rate 10)
  (dotimes (i 25)
    (ros::spin-once)
    (ros::sleep))
  )

(ros::roseus "add_two_ints_client")
(run-all-tests)

(exit)





