#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/test_add_two_ints.py
;;;

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

(deftest test-add-two-doubles
  (ros::wait-for-service "add_two_doubles")
  (dolist (test (list (cons 1.1 2.2) (cons 0.0 0.0) (cons -1.0 -2.0)
                      (cons 12312.12312 98023.98023)))
    (print test)
    (let ((a (car test)) (b (cdr test)))
      (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b)))
      (setq req (instance roseus::AddTwoDoublesRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_doubles" req))
      (assert (eps= (+ (send req :a) (send req :b)) (send res :sum))
              (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                           a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum))))
      (sys::gc)
      )))


(ros::roseus "add_two_doubles_client")
(run-all-tests)

(exit)
