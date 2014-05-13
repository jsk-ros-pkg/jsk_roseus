#!/usr/bin/env roseus
;;
;; /home/k-okada/ros/cturtle/stacks/geometry/tf/test/testPython.py
;;  rosrun tf static_transform_publisher 1 0 0 0 0 0 /MANDALAY /JUPITER 10


(require :unittest "lib/llib/unittest.l")
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

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))

  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)
  (setq tm (ros::time-now))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/JUPITER" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)
  (make-list 1000000)
  (assert (send l :can-transform "/MANDALAY" "/JUPITER" tm))
  (assert (send l :get-latest-common-time "MANDALAY" "JUPITER"))
  (setq c (send l :lookup-transform "MANDALAY" "JUPITER" tm))
  (assert c)
  (assert (eps= (norm (send c :difference-position c1)) 0))
  (assert (eps= (norm (send c :difference-rotation c1)) 0))

  ;;
  (setq vmrss (elt (unix::getrusage 0) 2))
  (format *error-output* "gc:~A, vmrss:~A~%" (sys::gc) vmrss)
  (assert (< vmrss 300000) "check memory leak")
  )

(deftest test-tf-lookup-cobject ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))

  ;; debug
  (dotimes (j 10)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "JUPITER" #i(0 0)))
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "TOKYO" #i(0 0)))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak cobject"))
  )

(deftest test-tf-lookup-cobject-time ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))

  ;; debug
  (dotimes (j 10)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "JUPITER" (send (ros::time) :sec-nsec)))
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "TOKYO" (send (ros::time) :sec-nsec)))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak cobject + rostime"))
  )

(deftest test-tf-lookup-cobject-time-makecoords-no-ret ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))

  ;; debug
  (dotimes (j 10)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "JUPITER" (send (ros::time) :sec-nsec))
      (setq ret #f(0 0 0 1 0 0 0))
      (make-cascoords :pos (float-vector
                            (* 1000.0 (elt ret 0))
                            (* 1000.0 (elt ret 1))
                            (* 1000.0 (elt ret 2)))
                      :rot
                      (user::quaternion2matrix
                       (float-vector (elt ret 3)
                                     (elt ret 4)
                                     (elt ret 5)
                                     (elt ret 6)))
                      :name "MANDALAY"))
    (dotimes (i 100000)
      (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "TOKYO" (send (ros::time) :sec-nsec))
      (setq ret #f(0 0 0 1 0 0 0))
      (make-cascoords :pos (float-vector
                            (* 1000.0 (elt ret 0))
                            (* 1000.0 (elt ret 1))
                            (* 1000.0 (elt ret 2)))
                      :rot
                      (user::quaternion2matrix
                       (float-vector (elt ret 3)
                                     (elt ret 4)
                                     (elt ret 5)
                                     (elt ret 6)))
                      :name "MANDALAY"))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak cobject + rostime + makecoords(dummy ret)"))
  )

(deftest test-tf-lookup-cobject-time-makecoords-ret ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))

  ;; debug
  (dotimes (j 5)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (setq ret (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "JUPITER" (send (ros::time) :sec-nsec)))
      (make-cascoords :pos (float-vector
                            (* 1000.0 (elt ret 0))
                            (* 1000.0 (elt ret 1))
                            (* 1000.0 (elt ret 2)))
                      :rot
                      (user::quaternion2matrix
                       (float-vector (elt ret 3)
                                     (elt ret 4)
                                     (elt ret 5)
                                     (elt ret 6)))
                      :name "MANDALAY"))

    (dotimes (i 100000)
      (setq ret (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "TOKYO" (send (ros::time) :sec-nsec)))
      (make-cascoords :pos (float-vector
                            (* 1000.0 (elt ret 0))
                            (* 1000.0 (elt ret 1))
                            (* 1000.0 (elt ret 2)))
                      :rot
                      (user::quaternion2matrix
                       (float-vector (elt ret 3)
                                     (elt ret 4)
                                     (elt ret 5)
                                     (elt ret 6)))
                      :name "MANDALAY"))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak cobject + rostime + makecoords(ret)"))
  )

(deftest test-tf-lookup-cobject-time-makecoords ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))

  ;; debug
  (dotimes (j 5)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (setq ret (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "JUPITER" (send (ros::time) :sec-nsec)))
      (when ret
       (make-cascoords :pos (float-vector
                             (* 1000.0 (elt ret 0))
                             (* 1000.0 (elt ret 1))
                             (* 1000.0 (elt ret 2)))
                       :rot
                       (user::quaternion2matrix
                        (float-vector (elt ret 3)
                                      (elt ret 4)
                                      (elt ret 5)
                                      (elt ret 6)))
                       :name "MANDALAY")))
    (dotimes (i 100000)
      (setq ret (ros::eustf-lookup-transform (l . ros::cobject)  "MANDALAY" "TOKYO" (send (ros::time) :sec-nsec)))
      (when ret
       (make-cascoords :pos (float-vector
                             (* 1000.0 (elt ret 0))
                             (* 1000.0 (elt ret 1))
                             (* 1000.0 (elt ret 2)))
                       :rot
                       (user::quaternion2matrix
                        (float-vector (elt ret 3)
                                      (elt ret 4)
                                      (elt ret 5)
                                      (elt ret 6)))
                       :name "MANDALAY")))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak cobject + rostime + makecoords"))
  )

(deftest test-tf-lookup ()
  (setq l (instance ros::transform-listener :init))
  (unix:sleep 1)

  (setq tm (ros::time))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" tm 1))
  (warning-message 2 "wait-for-transform at ~A returns ~A~%" tm ret)

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))
  ;;
  (dotimes (j 10)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 100000)
      (setq c (send l :lookup-transform "MANDALAY" "JUPITER" (ros::time)))
      (unless (and c (eps= (norm (send c :difference-position c1)) 0)
                     (eps= (norm (send c :difference-rotation c1)) 0))
        (assert (and c (eps= (norm (send c :difference-position c1)) 0) (eps= (norm (send c :difference-rotation c1)) 0)) "lookup-transform failed ~A ~A" c c1)
        ))
    (warning-message 2 "switch target to /JUPITER~%")
    (dotimes (i 100000)
      (setq c (send l :lookup-transform "MANDALAY" "TOKYO" (ros::time)))
      (unless (and c (eps= (norm (send c :difference-position c2)) 0)
                     (eps= (norm (send c :difference-rotation c2)) 0))
        (assert (and c (eps= (norm (send c :difference-position c2)) 0) (eps= (norm (send c :difference-rotation c2)) 0)) "lookup-transform failed ~A ~A" c c2)
        ))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak"))
  )

(deftest test-tf2-lookup ()
  (setq l (instance ros::buffer-client :init :namespace "tf2_buffer_server"))

  (let ((ret (send l :wait-for-server)))
    (unless ret
      (error "tf2 :wait-for-server failed~%")))

  (unix:sleep 1)
  (setq tm (ros::time-now))
  (setq ret (send l :wait-for-transform "/MANDALAY" "/TOKYO" (ros::time) 120))
  (warning-message 2 "wait-for-transform at ~A returns ~A (delayed ~A)~%" tm ret (ros::time- (ros::time-now) tm))

  (setq c1 (make-coords :pos #f(1000 0 0) :rpy #f(0 0 0)))
  (setq c2 (make-coords :pos #f(2000 2000 3000) :rpy #f(0.1 0.2 0.3)))
  ;;
  (dotimes (j 10)
    (warning-message 2 "switch target to /TOKYO~%")
    (dotimes (i 10)
      (setq tm (ros::time-now))
      (setq ret (send l :wait-for-transform "MANDALAY" "JUPITER" (ros::time) 120))
      (warning-message 2 "wait-for-transform at ~A returns ~A (delayed ~A)~%" tm ret (ros::time- (ros::time-now) tm))
      (setq tm (ros::time-now))
      (setq c (send l :lookup-transform "MANDALAY" "JUPITER" (ros::time) 120.0))
      (warning-message 2 "lookup-transform returns ~A (delayed ~A)~%" c (ros::time- (ros::time-now) tm))
      (unless (and c (eps= (norm (send c :difference-position c1)) 0)
                     (eps= (norm (send c :difference-rotation c1)) 0))
        (assert (and c (eps= (norm (send c :difference-position c1)) 0) (eps= (norm (send c :difference-rotation c1)) 0)) "tf2: lookup-transform failed ~A ~A" c c1)
        ))
    (warning-message 2 "switch target to /JUPITER~%")
    (dotimes (i 10)
      (setq tm (ros::time-now))
      (setq ret (send l :wait-for-transform "MANDALAY" "TOKYO" (ros::time) 120))
      (warning-message 2 "wait-for-transform at ~A returns ~A (delayed ~A)~%" tm ret (ros::time- (ros::time-now) tm))
      (setq tm (ros::time-now))
      (setq c (send l :lookup-transform "MANDALAY" "TOKYO" (ros::time) 120.0))
      (warning-message 2 "lookup-transform returns ~A (delayed ~A)~%" c (ros::time- (ros::time-now) tm))
      (unless (and c (eps= (norm (send c :difference-position c2)) 0)
                     (eps= (norm (send c :difference-rotation c2)) 0))
        (assert (and c (eps= (norm (send c :difference-position c2)) 0) (eps= (norm (send c :difference-rotation c2)) 0)) "tf2: lookup-transform failed ~A ~A" c c2)
        ))

    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak"))
  )

(run-all-tests)

(exit)
