#!/usr/bin/env roseus
;;
;; /home/k-okada/ros/cturtle/stacks/geometry/tf/test/testPython.py
;;  rosrun tf static_transform_publisher 1 0 0 0 0 0 /MANDALAY /JUPITER 10


(require :unittest "lib/llib/unittest.l")
;;(unix::putenv "ROSCONSOLE_CONFIG_FILE=/home/k-okada/ros/electric/jsk-ros-pkg/roseus/roseus.config")
(ros::roseus "test-tf")

;;(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(deftest test-tf->coords ()
  (let (m c)
    ;; test tf->transform -> coords
    (setq m (instance geometry_msgs::Transform :init))
    (send m :translation :y 5.0)
    (send m :rotation (ros::rot->tf-quaternion (unit-matrix 3)))
    (setq c (ros::tf-transform->coords m))
    (assert (v= (send c :worldpos) #f(0 5000 0)))
    (assert (v= (array-entity (send c :worldrot)) #f(1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0)))
    (setq m (instance geometry_msgs::TransformStamped :init))
    (send m :transform :translation :y 5.0)
    (send m :transform :rotation (ros::rot->tf-quaternion (unit-matrix 3)))
    (setq c (ros::tf-transform-stamped->coords m))
    (assert (v= (send c :worldpos) #f(0 5000 0)))
    (assert (v= (array-entity (send c :worldrot)) #f(1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0)))
    ))

(deftest test-rpy->quaternion-msg ()
  (let (q msg)
    (setq q (ros::create-identity-quaternion))
    (assert (v= q #f(1.0 0.0 0.0 0.0)))
    (setq q (ros::create-quaternion-from-rpy 0.1 0.2 0.3))
    (assert (eps= 0.0 (norm (v- q (matrix2quaternion (rpy-matrix 0.3 0.2 0.1))))))
    (setq q (matrix2quaternion (rpy-matrix -0.1 -0.2 -0.3)))
    (setq msg (ros::create-quaternion-msg-from-rpy -0.3 -0.2 -0.1))
    (assert (and (eps= (send msg :x) (elt q 1))
                 (eps= (send msg :y) (elt q 2))
                 (eps= (send msg :z) (elt q 3))
                 (eps= (send msg :w) (elt q 0))))
    (setq msg (ros::identity-quaternion))
    (assert (and (= (send msg :x) 0.0)
                 (= (send msg :y) 0.0)
                 (= (send msg :z) 0.0)
                 (= (send msg :w) 1.0)))
    ))

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
      (ros::ros-info "wait-for-transform at ~A returns ~A (delayed ~A)" tm ret (ros::time- (ros::time-now) tm))
      (setq tm (ros::time-now))
      (setq c (send l :lookup-transform "MANDALAY" "JUPITER" (ros::time) 120.0))
      (ros::ros-info "lookup-transform returns ~A (delayed ~A)" c (ros::time- (ros::time-now) tm))
      (unless (and c (eps= (norm (send c :difference-position c1)) 0)
                     (eps= (norm (send c :difference-rotation c1)) 0))
        (assert (and c (eps= (norm (send c :difference-position c1)) 0) (eps= (norm (send c :difference-rotation c1)) 0)) "tf2: lookup-transform failed ~A ~A" c c1)
        ))
    (warning-message 2 "switch target to /JUPITER~%")
    (dotimes (i 10)
      (setq tm (ros::time-now))
      (setq ret (send l :wait-for-transform "MANDALAY" "TOKYO" (ros::time) 120))
      (ros::ros-info "wait-for-transform at ~A returns ~A (delayed ~A)" tm ret (ros::time- (ros::time-now) tm))
      (setq tm (ros::time-now))
      (setq c (send l :lookup-transform "MANDALAY" "TOKYO" (ros::time) 120.0))
      (ros::ros-info "lookup-transform returns ~A (delayed ~A)" c (ros::time- (ros::time-now) tm))
      (unless (and c (eps= (norm (send c :difference-position c2)) 0)
                     (eps= (norm (send c :difference-rotation c2)) 0))
        (assert (and c (eps= (norm (send c :difference-position c2)) 0) (eps= (norm (send c :difference-rotation c2)) 0)) "tf2: lookup-transform failed ~A ~A" c c2)
        ))

    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A~%" j (sys::gc) vmrss)
    (assert (< vmrss 300000) "check memory leak"))
  )

(deftest test-tf-frame-exists ()
  (let ((b (instance ros::transform-broadcaster :init))
        (l (instance ros::transform-listener :init))
        (tm (ros::time-now)))
    (send b :send-transform (make-coords) "world" "turtle" tm)
    (send l :wait-for-transform "world" "turtle" tm 0.5)
    (assert (send l :frame-exists "world") "tf-transformer: frame-exists failed. t expected")
    (assert (send l :frame-exists "turtle") "tf-transformer: frame-exists failed. t expected")
    (assert (not (send l :frame-exists "rabbit")) "tf-transformer: frame-exists failed. nil expected"))
  )

;; check set-transform bug https://github.com/jsk-ros-pkg/jsk_roseus/pull/649
(deftest test-set-tranform-transformer ()
  (let (f m c1)
    (setq f (instance ros::transformer :init))
    (setq m (instance geometry_msgs::TransformStamped :init))
    (send m :header :frame_id "this_frame_a")
    (send m :child_frame_id "child_a")
    (send m :transform :translation :x 1) ;; 1000[mm]
    (send m :transform :translation :y 2)
    (send m :transform :translation :z 3)
    (send m :transform :rotation (ros::rot->tf-quaternion (unit-matrix 3)))
    (ros::ros-info "TransformStamped ~A ~A ~A / ~A ~A ~A ~A"
                   (send m :transform :translation :x)
                   (send m :transform :translation :y)
                   (send m :transform :translation :z)
                   (send m :transform :rotation :x)
                   (send m :transform :rotation :y)
                   (send m :transform :rotation :z)
                   (send m :transform :rotation :w))
    (setq c1 (ros::tf-transform-stamped->coords m))
    (ros::ros-info "tf-transform-stamped->coords -> ~A" c1)
    (send f :set-transform c1)
    (ros::ros-info "get-frame-strings ~A" (send f :get-frame-strings))
    (ros::ros-info "this_frame_a -> child_a ~A" (setq c2 (send f :lookup-transform "this_frame_a" "child_a" (ros::time 0))))
    (assert (and c2
                 (eps= (norm (send c1 :difference-position c2)) 0.0)
                 (eps= (norm (send c1 :difference-rotation c2)) 0.0))
            (format nil "position ~A, rotation ~A"
                    (and c2 (send c1 :difference-position c2))
                    (and c2 (send c1 :difference-rotation c2))))

    (setq c1 (send c1 :inverse-transformation)) ;; set c^(-1)
    (ros::ros-info "child_a -> this_frame_a ~A" (setq c2 (send f :lookup-transform "child_a" "this_frame_a" (ros::time 0))))
    (assert (and c2
                 (eps= (norm (send c1 :difference-position c2)) 0.0)
                 (eps= (norm (send c1 :difference-rotation c2)) 0.0))
            (format nil "position ~A, rotation ~A"
                    (and c2 (send c1 :difference-position c2))
                    (and c2 (send c1 :difference-rotation c2))))
    ))

;; check set-transform bug https://github.com/jsk-ros-pkg/jsk_roseus/pull/649
(deftest test-set-transform-listener-roadcaster ()
  (let (b l c1 c2)
    (setq b (instance ros::transform-broadcaster :init))
    (setq l (instance ros::transform-listener :init))
    (setq c1 (make-coords :pos #f(1 2 3) :rpy #f(0.1 0.2 0.3)))
    (ros::ros-info "coords -> ~A" c1)
    (send b :send-transform c1 "this_frame_b" "child_b" (ros::time 0))
    (send l :wait-for-transform "this_frame_b" "chidl_b" (ros::time 0) 1)
    (ros::ros-info "get-frame-strings ~A" (send l :get-frame-strings))
    ;;
    (ros::ros-info "this_frame_b -> child_b ~A" (setq c2 (send l :lookup-transform "this_frame_b" "child_b" (ros::time 0))))
    (assert (and c2
                 (eps= (norm (send c1 :difference-position c2)) 0.0)
                 (eps= (norm (send c1 :difference-rotation c2)) 0.0))
            (format nil "position ~A, rotation ~A"
                    (and c2 (send c1 :difference-position c2))
                    (and c2 (send c1 :difference-rotation c2))))

    (setq c1 (send c1 :inverse-transformation)) ;; set c^(-1)
    (ros::ros-info "child_b -> this_frame_b ~A" (setq c2 (send l :lookup-transform "child_b" "this_frame_b" (ros::time 0))))
    (assert (and c2
                 (eps= (norm (send c1 :difference-position c2)) 0.0)
                 (eps= (norm (send c1 :difference-rotation c2)) 0.0))
            (format nil "position ~A, rotation ~A"
                    (and c2 (send c1 :difference-position c2))
                    (and c2 (send c1 :difference-rotation c2))))
    ))

(run-all-tests)

(exit)
