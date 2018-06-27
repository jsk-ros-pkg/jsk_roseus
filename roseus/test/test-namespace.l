#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-namespace")

;(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(deftest test-namespace ()
  (let ((ns (ros::get-namespace))
        (ans (ros::get-param "~namespace")))
    (ros::ros-info "get-namespace ~A, expected result is ~A" ns ans)
    (assert (equal ns ans) (format nil "get-namespace ~A, expected result is ~A" ns ans))
    ))
  
(run-all-tests)

(exit)
