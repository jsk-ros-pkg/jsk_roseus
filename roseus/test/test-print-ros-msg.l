#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")

(init-unit-test)


(deftest test-print-ros-msg ()  
  (let ((result-string
         (with-output-to-string (s)
           (let ((msg (ros::coords->tf-pose (make-coords :pos #f(1.0 2.0 3.0))))
                 (*standard-output* s))
             (print-ros-msg msg))))
        (expected-string
":position -- geometry_msgs::point
  :x
  0.001
  :y
  0.002
  :z
  0.003
:orientation -- geometry_msgs::quaternion
  :x
  0.0
  :y
  0.0
  :z
  0.0
  :w
  1.0
"":position -- geometry_msgs::point
  :x
  0.001
  :y
  0.002
  :z
  0.003
:orientation -- geometry_msgs::quaternion
  :x
  0.0
  :y
  0.0
  :z
  0.0
  :w
  1.0
"))
    (assert (string= result-string expected-string))))


(run-all-tests)

(exit)
