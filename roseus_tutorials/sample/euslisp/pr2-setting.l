#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(setq *input-bbox-array* (ros::get-param "~/input_bbox_array" "/HSI_color_filter/boxes"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "/base_footprint"))
(setq *camera-frame-id* (ros::get-param "~camera_frame_id" ":head_mount_kinect_rgb_optical_frame_lk"))
(pr2-init)
(setq *robot* *pr2*)
(load "package://roseus_tutorials/src/display-tf-bounding-box-array.l")
