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

(deftest test-master ()

  (ros::ros-info "get-host ~A" (ros::get-host))
  (ros::ros-info "get-nodes ~A" (ros::get-nodes))
  (ros::ros-info "get-port ~A" (ros::get-port))
  (ros::ros-info "get-uri ~A" (ros::get-uri))
  (ros::ros-info "get-topics ~A" (ros::get-topics))

  (assert (equal (ros::get-nodes) '("/eusroseus" "/rosout"))) ;; test-roseus.test's test-name
  (assert (equal (ros::get-uri) (format nil "http://~A:~A/" (ros::get-host) (ros::get-port))))
  (assert (equal (ros::get-topics) '(("/rosout" . "rosgraph_msgs/Log") ("/rosout_agg" . "rosgraph_msgs/Log"))))

  )

(run-all-tests)

(exit)
