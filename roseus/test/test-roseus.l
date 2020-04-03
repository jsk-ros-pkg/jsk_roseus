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

(deftest test-master ()

  (ros::ros-info "get-host ~A" (ros::get-host))
  (ros::ros-info "get-nodes ~A" (ros::get-nodes))
  (ros::ros-info "get-port ~A" (ros::get-port))
  (ros::ros-info "get-uri ~A" (ros::get-uri))
  (ros::ros-info "get-topics ~A" (ros::get-topics))

  (format *error-output* "get-nodes : ~A~%" (ros::get-nodes))
  (format *error-output* "get-uri : ~A, get-host : ~A, get-port : ~A~%" (ros::get-uri) (ros::get-host) (ros::get-port))
  (format *error-output* "get-topics ~A~%~%" (ros::get-topics))

  ;; from roslaunch 1.12.8 https://github.com/ros/ros_comm/issues/1097, rosout node is not launched
  ;; order or (get-nodes) is not determined
  (assert (member "/eusroseus" (ros::get-nodes) :test #'string=)
          "get-nodes : ~A" (ros::get-nodes)) ;; test-roseus.test's test-name
  ;; setup.sh sets http://host:port but rostest sets http://host:port/, remove tailing /
  (assert (string= (string-right-trim "/" (ros::get-uri)) (format nil "http://~A:~A" (ros::get-host) (ros::get-port)))
          "get-uri : ~A, get-host : ~A, get-port : ~A" (ros::get-uri) (ros::get-host) (ros::get-port))
  ;; order or (get-topics) is not determined
  (assert (member '("/rosout" . "rosgraph_msgs/Log") (ros::get-topics) :test #'equal)
          "get-topics : ~A" (ros::get-topics))

  )

(ros::load-ros-manifest "std_msgs")
(defun equal-vector-list (a b &optional (eps *epsilon*))
  (every #'(lambda (x y) (eps-v= x y eps)) a b))
(deftest test-marker-msg ()
  (let (b)
    (assert (equal (vector->rgba #f(255 255 0)) (instance std_msgs::ColorRGBA :init :r 255 :g 255 :b 0 :a 1)))
    (setq b (marker-msg->shape/cylinder (cylinder->marker-msg (make-cylinder 100 100) (instance std_msgs::header))))
    (assert (equal-vector-list (send b :vertices) (send (make-cylinder 100 100) :vertices)))
    (setq b (marker-msg->shape/cube (cube->marker-msg (make-cube 100 100 100) (instance std_msgs::header))))
    (assert (equal-vector-list (send b :vertices) (send (make-cube 100 100 100) :vertices)))
    (setq b (marker-msg->shape/sphere (sphere->marker-msg (make-sphere 100) (instance std_msgs::header))))
    (assert (equal-vector-list (send b :vertices) (send (make-sphere 100) :vertices)))
    (setq b (line->marker-msg (list #f(0 0 0) #f(100 100 100)) (instance std_msgs::header)))
    (assert (equal-vector-list (mapcar #'(lambda (l) (ros::tf-point->pos l)) (send b :points)) (list #f(0 0 0) #f(100 100 100))))
    (setq b (line-list->marker-msg (list #f(0 0 0) #f(100 100 100)) (instance std_msgs::header)))
    (assert (equal-vector-list (mapcar #'(lambda (l) (ros::tf-point->pos l)) (send b :points)) (list #f(0 0 0) #f(100 100 100))))
    ;;
    (assert (faces->marker-msg (send (make-cube 100 100 100) :faces) (instance std_msgs::header)))
    (assert (object->marker-msg (make-cube 100 100 100) (instance std_msgs::header)))
    (assert (wireframe->marker-msg (make-cube 100 100 100) (instance std_msgs::header)))
    (assert (text->marker-msg "test" (make-coords) (instance std_msgs::header)))
    (assert (coords->marker-msg (make-coords) (instance std_msgs::header)))
    ;; (mesh->marker-msg)
    (when (functionp 'make-random-pointcloud) ;; only available on jskeus >= 1.0.9
      (let ((pc (make-random-pointcloud :num 100 :with-color t :with-normal t)))
        (assert (pointcloud->marker-msg pc (instance std_msgs::header))))
      (let ((pc (make-random-pointcloud :num 100 :with-color t :with-normal t))
            msg)
        (assert
         (setq msg (make-ros-msg-from-eus-pointcloud pc)))
        (let ((rt (make-eus-pointcloud-from-ros-msg msg)))
          (assert (= (send pc :size) (send rt :size)))
          (assert (equal-vector-list (send pc :point-list) (send rt :point-list)))
          ;; color conversion ( float -> uint8 -> float ) may change color value
          (assert (equal-vector-list (send pc :color-list) (send rt :color-list) 0.007))
          (assert (equal-vector-list (send pc :normal-list) (send rt :normal-list)))
          )))
    (assert (eusobj->marker-msg (make-cube 100 100 100) (instance std_msgs::header)))
    (assert (arrow->marker-msg (make-coords) (instance std_msgs::header)))
    (assert (marker-msg->shape (cylinder->marker-msg (make-cylinder 100 100) (instance std_msgs::header))))
    (assert (marker-msg->shape (cube->marker-msg (make-cube 100 100 100) (instance std_msgs::header))))
    (assert (marker-msg->shape (sphere->marker-msg (make-sphere 100) (instance std_msgs::header))))
    (assert (shape-msg->shape (cylinder->shape-msg (make-cylinder 100 100))))
    (assert (shape-msg->shape (cube->shape-msg (make-cube 100 100 100))))
    (assert (shape-msg->shape (sphere->shape-msg (make-sphere 100))))
    ;;
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

(deftest test-image-conversion ()
  (when (and
         (assoc :create-viewer (send camera-model :methods))
         (probe-file (format nil "~A/irteus/demo/sample-camera-model.l" *eusdir*)))
    (load "irteus/demo/sample-camera-model.l")
    (sample-get-camera-image-2)
    (setq sndimg (send *camera-model* :get-image))
    (setq rcvimg nil)
    (setq seq 0)
    (ros::advertise "test_image" sensor_msgs::Image 5)
    (unix:sleep 2)
    (ros::rate 1)

    (ros::subscribe "test_image" sensor_msgs::Image
                    #'(lambda (m)
                        (setq rcvimg (ros::sensor_msgs/Image->image m))))

    (while (and (ros::ok) (null rcvimg))
      (setq msg (ros::image->sensor_msgs/Image sndimg :seq (inc seq) :frame_id "hoge"))
      (ros::publish "test_image" msg)
      (ros::spin-once)
      (ros::sleep))

    (assert rcvimg "received image")
    (assert (eq (send rcvimg :width) (send sndimg :width)) "eq image :width")
    (assert (eq (send rcvimg :height) (send sndimg :height)) "eq image :height")
    (assert (string= (send rcvimg :entity) (send sndimg :entity)) "eq image :entity")
    ))

(run-all-tests)

(exit)
