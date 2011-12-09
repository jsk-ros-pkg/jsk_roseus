#!/usr/bin/env roseus

(load "unittest.l")

(ros::roseus "test-disconnect")

(setq *m* nil)
(defun cb (m) (setq *m* m))
(ros::subscribe "/string" std_msgs::String #'cb)

(init-unit-test)

(deftest test-common ()
  (dotimes (i 5)
    (unix::sleep 1)
    (when *m* (return)))
  (unix::system "rosnode kill publisher")
  (unix::sleep 1)
  (assert *m*)
  )

(run-all-tests)

(exit)
