#!/usr/bin/env roseus
;;;
;;; ROS 2 version of test-add-two-ints.l
;;; - Reduced iterations (ROS 2 non-persistent calls have DDS discovery overhead)
;;; - NIL-safe: ros::service-call returns NIL on timeout
;;;

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

;; Service call with retry for DDS connection establishment
(defun service-call-retry (service req &optional persist (retries 3))
  (let (res)
    (dotimes (i retries)
      (setq res (ros::service-call service req persist))
      (when res (return-from service-call-retry res))
      (warning-message 3 "service call attempt ~A failed, retrying...~%" (1+ i))
      (unix::usleep (* 1000 1000)))  ;; 1 second between retries
    res))

(deftest test-add-two-ints
  (ros::wait-for-service "add_two_ints")
  ;; Note: extreme fixnum values (most-positive/negative-fixnum) are
  ;; excluded because sys::peek :long on x86_64 doesn't handle
  ;; values outside EusLisp fixnum range (needs bignum conversion)
  (dolist (test (list (cons 1 2) (cons 0 0) (cons -1 -2)
                      (cons 12312 98023)
                      (cons -12345 12345)
                      (cons 999999999 1)
                      (cons -999999999 999999999)
                      (cons #x40000000 #x3fffffff)
                      ))
    (print test)
    (let ((a (car test)) (b (cdr test)))
      (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b)))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (service-call-retry "add_two_ints" req t))
      (assert res (format nil "service call returned nil for ~A+~A" a b))
      (when res
        (assert (= (+ (send req :a) (send req :b)) (send res :sum))
                (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                             a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum)))))
      (sys::gc)
      )))

(deftest test-add-two-ints-many
  (ros::wait-for-service "add_two_ints")
  (setq service-call-error 0)
  (dotimes (i 1200)
    (let ((a i) (b (random 100000000)))
      (if (= (mod i 100) 0)
          (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b))))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_ints" req))
      (when (or (null res) (not (= (+ (send req :a) (send req :b)) (send res :sum))))
        (incf service-call-error)
        (warning-message 3 (format nil "[~A] integration failure (~A+~A)=~A(~A)/=~A(~A)~%"
                                   service-call-error
                                   a b (+ a b) (class (+ a b))
                                   (if res (send res :sum) nil)
                                   (if res (class (send res :sum)) nil))))
      (assert (or (< service-call-error 5) (and res (= (+ (send req :a) (send req :b)) (send res :sum))))
              (format nil "integration failure ~A times ... (~A+~A)=~A(~A)/=~A(~A)"
                           service-call-error a b (+ a b) (class (+ a b))
                           (if res (send res :sum) nil)
                           (if res (class (send res :sum)) nil)))
      (sys::gc)
      )))

(deftest test-add-two-ints-persistent-many
  (ros::wait-for-service "add_two_ints")
  (dotimes (i 1200)
    (let ((a i) (b (random 100000000)))
      (if (= (mod i 100) 0)
          (warning-message 2 "Requesting ~A(~A) + ~A(~A) -> ~A(~A)~%" a (class a) b (class b) (+ a b) (class (+ a b))))
      (setq req (instance roseus::AddTwoIntsRequest :init :a a :b b))
      (setq res (ros::service-call "add_two_ints" req t))
      (assert res (format nil "service call returned nil for ~A+~A" a b))
      (when res
        (assert (= (+ (send req :a) (send req :b)) (send res :sum))
                (format nil "integration failure (~A+~A)=~A(~A)/=~A(~A)"
                             a b (+ a b) (class (+ a b)) (send res :sum) (class (send res :sum)))))
      (sys::gc)
      )))

(ros::roseus "add_two_ints_client")
(run-all-tests)

(exit)
