#!/usr/bin/env roseus
;;
;; /home/k-okada/ros/cturtle/stacks/geometry/tf/test/testPython.py
;;  rosrun tf static_transform_publisher 1 0 0 0 0 0 /MANDALAY /JUPITER 10


(load "unittest.l")
;;(unix::putenv "ROSCONSOLE_CONFIG_FILE=/home/k-okada/ros/electric/jsk-ros-pkg/roseus/roseus.config")
(ros::roseus "test-tf")

(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(deftest test-common ()
  (setq m (instance geometry_msgs::TransformStamped :init))
  (send m :header :frame_id "PARENT")
  (send m :child_frame_id "THISFRAME")
  (send m :transform :translation :y 5.0)
  (send m :transform :rotation (ros::rot->tf-quaternion (unit-matrix 3)))
  (setq f (instance ros::transformer :init))
  (setq c (ros::tf-transform-stamped->coords m))
  (send f :set-transform c)
  (setq afs (send f :all-frames-as-string))
  (warning-message 2 "all frame as string=~A~%" afs)
  (assert afs)
  (assert (substringp "PARENT" afs))
  (assert (substringp "THISFRAME" afs))
  (setq tm (send f :get-latest-common-time "THISFRAME" "PARENT"))
  (warning-message 2 "common time = ~A~%" tm)
  (assert (= (send tm :to-sec) 0.0))

  (dolist (ti (list 3 5 10 11 19 20 21))
    (send c :stamp (integer-vector ti 0))
    (send f :set-transform c)
    (setq tm (send f :get-latest-common-time "THISFRAME" "PARENT"))
    (warning-message 2 "common time = ~A~%" tm)
    (assert (= (send tm :to-sec) ti))
    )

  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)
  (setq ret (send l :wait-for-transform "/MANDALAY" "/JUPITER" (ros::time-now) 1))
  (warning-message 2 "wait-for-transform ~A~%" ret)
  (make-list 1000000)
  (assert (send l :wait-for-transform "/MANDALAY" "/JUPITER" (ros::time-now) 1))
  (assert (send l :can-transform "/MANDALAY" "/JUPITER" (ros::time-now)))
  (assert (send l :get-latest-common-time "MANDALAY" "JUPITER"))
  (assert (send l :lookup-transform "MANDALAY" "JUPITER" (ros::time)))
  )

(run-all-tests)

(exit)
