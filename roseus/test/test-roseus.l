#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-roseus")

;(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(deftest test-time ()
  (dotimes (j 20)
    (dotimes (i 100000)
      (ros::time))

    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "gc:~A, vmrss:~A~%" (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak")
    ))

(run-all-tests)

(exit)
