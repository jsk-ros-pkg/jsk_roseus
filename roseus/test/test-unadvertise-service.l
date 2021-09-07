#!/usr/bin/env roseus
;;;
;;;

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

(defun empty-cb (req)
  (send req :response))

(deftest test-unadvertise-service
  (ros::roseus "add_two_ints_server")
  (ros::advertise-service "empty_service" std_srvs::Empty #'empty-cb)
  (ros::unadvertise-service "empty_service"))

(ros::roseus "add_two_ints_client")
(run-all-tests)

(exit)
