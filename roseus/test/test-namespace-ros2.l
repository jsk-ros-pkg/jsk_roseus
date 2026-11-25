#!/usr/bin/env roseus
;;
;; ROS 2 version of test-namespace.l
;; Expected namespace is passed as the first argument.
;; Usage: roseus test-namespace-ros2.l /
;;        roseus test-namespace-ros2.l /ns/ --ros-args -r __ns:=/ns

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-namespace")

;; Get expected namespace from command line argument
(setq expected-ns (elt lisp::*eustop-argument*
                       (1+ (position "test-namespace-ros2.l" lisp::*eustop-argument*
                                     :test #'substringp))))

(init-unit-test)

(deftest test-namespace ()
  (let ((ns (ros::get-namespace)))
    (ros::ros-info "get-namespace ~A, expected result is ~A" ns expected-ns)
    (assert (equal ns expected-ns) (format nil "get-namespace ~A, expected result is ~A" ns expected-ns))
    ))

(run-all-tests)

(exit)
