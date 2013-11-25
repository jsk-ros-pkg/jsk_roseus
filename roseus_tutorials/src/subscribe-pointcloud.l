#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus_tutorials")

(setq ros::*compile-message* t)
(ros::roseus-add-msgs "sensor_msgs")

(setq *points* nil)
(defun callback ( msg )
  ;; ROSのメッセージをeuslispのpointcloudに変換
  (setq *points*
        (make-eus-pointcloud-from-ros-msg
         msg :remove-nan t)) ;; for replace -> :remove-nan :replace
  (if (and *points* (> (send *points* :size) 0))
      (setf (get *points* :header) (send msg :header))
    (setq *points* nil))
  *points*)

(ros::roseus "subscribe_pointcloud")

(ros::defrosparam *frame-id* "~centroid_frame" "/points_centroid")

(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::subscribe "/camera/depth_registered/points"
                sensor_msgs::PointCloud2
                #'callback)

(defun start-subscribe ()
  (ros::rate 100)
  (do-until-key
   (setq *points* nil)
   (ros::spin-once)
   ;; *points*にpointcloudが入っている
   (when *points*
     (let* ((pos (send *points* :centroid))
            (cds (make-coords :pos pos)))
       (send *tfb* :send-transform cds
             (send (get *points* :header) :frame_id)
             *frame-id*
             (send (get *points* :header) :stamp)))
     )
   (ros::sleep)
   ))

(start-subscribe)
