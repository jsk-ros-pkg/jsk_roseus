#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
;;;
;;;
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(defvar *root-frame-id* "/camera")
;;(setq *target-object* (iphone4 :name "/test_object"))
(objects (list (make-cube 1000 1000 1) (make-cylinder 10 1000) *target-object*))

(ros::roseus "objectdetection_client")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start checkerboard-detector.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./checkerboard-detector.l"))

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))
(defun tf-object-update (objs &key (root-frame-id "/base_footprint"))
  (let (c msg obj-frame-id)
    (dolist (obj objs)
      (setq obj-frame-id (send obj :name))
      ;; advertise
      (unless (get obj :marker-advertised)
        (setf (get obj :marker-advertised) t)
        (ros::ros-info "ADVERTISE ~A" obj-frame-id)
        (ros::advertise obj-frame-id visualization_msgs::Marker 5))
      ;; get coordinates
      (setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
      ;; send marker
      (when (and (null (setq msg (get obj :marker-message)))
                 (> (ros::get-num-subscribers obj-frame-id) 0))
        (setf (get obj :marker-message)
              (setq msg
                    (object->marker-msg
                     obj
                     (instance std_msgs::header :init :stamp (ros::time-now)
                               :frame_id obj-frame-id))))
        (ros::ros-info "PUBLISH ~A" obj-frame-id)
        (ros::publish obj-frame-id msg))
      ;; update eus object
      (if c (send obj :move-to c :world)))
    ))

(defun objectdetection-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) frame-id type ret)
    (setq frame-id (concatenate string "/" (send msg :header :frame_id)))
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list type))
      (send *tfb* :send-transform ret frame-id type)
      (ros::ros-info "~A ~A ~A" ret frame-id type)
      (ros::publish "image_marker" mrk))
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(ros::rate 10)
(while (ros::ok)
  (tf-object-update (list *target-object*) :root-frame-id *root-frame-id*)
  (ros::spin-once)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (ros::sleep)
  )

