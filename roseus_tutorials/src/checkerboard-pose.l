#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
;;;
;;;
(ros::roseus "checkerboard_client")
(if (string= (ros::get-namespace) "/")
    (ros::ros-warn ";; Started in the global namespace! This is probably wrong.~%Start checkerboard-pose.l in the camera namespace.~%Example command-line usage: $ ROS_NAMESPACE=my_camera ./checkerboard-pose.l"))

(defun checkerboard-cb (pose)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) ret)
    (print (send pose :pose))
    (setq ret (ros::tf-pose->coords (send pose :pose)))
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames (list "/checkerboard_pose_frame"))
    (send tf-broadcaster :send-transform
          ret (send pose :header :frame_id) "/checkerboard_pose_frame")
    (ros::ros-info "~A" ret)
    (ros::publish "image_marker" mrk)
    ))

(setq tf-broadcaster (instance ros::transform-broadcaster :init))
(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "board_pose" geometry_msgs::PoseStamped #'checkerboard-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )

