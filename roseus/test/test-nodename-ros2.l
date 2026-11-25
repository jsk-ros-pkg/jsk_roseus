#!/usr/bin/env roseus
;;
;; ROS 2 version of test-nodename.l
;; Node name is passed as the first argument (no __name:= parsing needed in ROS 2)

(require :unittest "lib/llib/unittest.l")

;; Get node name from command line argument
(setq node-name (elt lisp::*eustop-argument*
                     (1+ (position "test-nodename-ros2.l" lisp::*eustop-argument*
                                   :test #'substringp))))

;;
(ros::roseus node-name :anonymous nil)

;; start test
(init-unit-test)

(deftest test-nodename ()

  (ros::ros-info "get-name ~A" (ros::get-name))

  ;; ROS 2 node names cannot contain hyphens, they are replaced with underscores
  (let ((expected-name (substitute #\_ #\- node-name)))
    (assert (string= (ros::get-name) (format nil "/~A" expected-name))
            (format nil "~A == /~A" (ros::get-name) expected-name)))
  )

(run-all-tests)

(exit)
