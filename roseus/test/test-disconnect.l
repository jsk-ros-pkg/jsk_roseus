#!/usr/bin/env roseus

(load "unittest.l")

(ros::roseus "test-disconnect")

(defun cb (m) (setq *m* m))
(ros::subscribe "/string" std_msgs::String #'cb)

(init-unit-test)

(deftest test-common ()
  (unix::sleep 5)
  (unix::system "rosnode kill publisher")
  (unix::sleep 1)
  t )

(run-all-tests)

(exit)
