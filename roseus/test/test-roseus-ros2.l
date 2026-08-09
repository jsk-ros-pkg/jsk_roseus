#!/usr/bin/env roseus
;;
;; ROS 2 version of test-roseus.l
;; Excluded: test-master (ROS 1 specific), test-marker-msg (roseus-utils.l),
;;           test-image-conversion (sensor_msgs + eustf.l + xvfb)

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-roseus")

(init-unit-test)

(deftest test-time ()
  (dotimes (j 20)
    (dotimes (i 100000)
      (ros::time))

    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "gc:~A, vmrss:~A~%" (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak")
    ))

(deftest test-sleep ()
  (let (d tm0 tm1)
    (setq tm0 (ros::time-now))
    (send (instance ros::duration :init 1.5) :sleep)
    (setq tm1 (ros::time-now))
    (assert (eps= (send (ros::time- tm1 tm0) :to-sec) 1.5 0.2))

    (setq tm0 (ros::time-now))
    (send (instance ros::duration :init 2 500000000) :sleep)
    (setq tm1 (ros::time-now))
    (assert (eps= (send (ros::time- tm1 tm0) :to-sec) 2.5 0.2))

    (setq tm0 (ros::time-now))
    (ros::duration-sleep 0.5)
    (setq tm1 (ros::time-now))
    (assert (eps= (send (ros::time- tm1 tm0) :to-sec) 0.5 0.2))
    ))

(deftest resolve-path ()
  (assert (ros::resolve-ros-path "package://roseus"))
  (assert (not (ros::resolve-ros-path "package://not_existing_package")))
  )

(deftest load-pathname ()
  (let ((fname (concatenate string #+:linux "/tmp/" "tmp-load-pathname-test.l")))
    (with-open-file (file fname :direction :output)
      (format file "(print \"Loaded.\")"))
    (assert (eq (load (pathname fname)) t))))

(run-all-tests)

(exit)
