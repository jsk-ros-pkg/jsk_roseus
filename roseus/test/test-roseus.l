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
    ;;
    ))

(deftest resolve-path ()
  (assert (ros::resolve-ros-path "package://roseus"))
  (assert (not (ros::resolve-ros-path "package://not_existing_package")))
  )

(run-all-tests)

(exit)
