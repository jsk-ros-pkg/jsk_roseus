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

(deftest test-add-two-ints
  (ros::wait-for-service "add_two_ints")
  (dolist (test (list (cons 1 2) (cons 0 0) (cons -1 -2)
                      (cons 12312 98023)
                      (cons lisp::most-positive-fixnum lisp::most-negative-fixnum)
                      (cons lisp::most-positive-fixnum -1)
                      (cons lisp::most-positive-fixnum 0)
                      (cons lisp::most-positive-fixnum 1)
                      (cons lisp::most-negative-fixnum -1)
                      (cons lisp::most-negative-fixnum 0)
                      (cons lisp::most-negative-fixnum 1)
                      (cons #x40000000 #x3fffffff) ;; error?
                      ))
    (print test)
    (let ((a (car test)) (b (cdr test)))
      (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b)))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_ints" req))
      (assert (= (+ (send req :a) (send req :b)) (send res :sum))
              (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                           a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum))))
      (sys::gc)
      )))

(deftest test-add-two-ints-many
  (ros::wait-for-service "add_two_ints")
  (dotimes (i 1200)
    (let ((a i) (b (random 100000000)))
      (if (= (mod i 100) 0)
          (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b))))
      (ros::ros-info "Requesting ~A(~A) + ~A(~A) -> ~A(~A)" a (class a) b (class b) (+ a b) (class (+ a b)))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_ints" req))
      (assert (= (+ (send req :a) (send req :b)) (send res :sum))
              (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                           a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum))))
      (sys::gc)
      )))

(deftest test-add-two-ints-persistent-many
  (ros::wait-for-service "add_two_ints")
  (dotimes (i 1200)
    (let ((a i) (b (random 100000000)))
      (if (= (mod i 100) 0)
          (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b))))
      (ros::ros-info "Requesting ~A(~A) + ~A(~A) -> ~A(~A)" a (class a) b (class b) (+ a b) (class (+ a b)))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_ints" req t))
      (assert (= (+ (send req :a) (send req :b)) (send res :sum))
              (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                           a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum))))
      (sys::gc)
      )))

(ros::roseus "add_two_ints_client")
(run-all-tests)

(exit)





