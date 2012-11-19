#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus_tutorials")

(setq ros::*compile-message* t)
(ros::roseus-add-msgs "sensor_msgs")

(setq *points* nil)
(defun callback ( msg )
  (setq *points*
        (make-eus-pointcloud-from-ros-msg
         msg :remove-nan t)) ;; for replace -> :remove-nan :replace
  (setf (get *points* :header) (send msg :header))
  *points*)

(ros::roseus "subscribe_pointcloud")

(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::subscribe "/camera/depth_registered/points"
                geometry_msgs::PointCloud2
                #'callback)

(ros::rate 100)
(do-until-key
 (setq *points* nil)
 (ros::spin-once)
 (when *points*
   (let ((pos (send *points* :centroid))
         (cds (make-coords :pos pos)))
     (send *tfb* :send-transform cds
           *base-frame* "/points_centroid"
           (send (get *points* :header) :stamp)))
   )
 (ros::sleep)
 )
