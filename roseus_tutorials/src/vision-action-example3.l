#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "sound_play")
(ros::load-ros-manifest "face_detector_mono")
;;;
;;;

;; vision callback
(defun checkerboard-cb (pose)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
	cam target-local target-world)
    (setq target-local (ros::tf-pose->coords (send pose :pose)))

    ;; for display
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames (list "/checkerboard_pose_frame"))
    (send *tfb* :send-transform
          target-local (send pose :header :frame_id) "/checkerboard_pose_frame")
    (ros::ros-info "~A" target-local)
    (ros::publish "image_marker" mrk)
    ;;
    ;; calc world coordinates of target object pose
    (setq cam (send *pr2* :camera "wide_stereo/right"))
    (setq target-world (send target-local :transform (send cam :worldcoords) :parent))

    ;; reach to the target
    (send *pr2* :rarm :inverse-kinematics
	  target-world :rotation-axis nil
	  :stop 3 :revert-if-fail nil :warnp nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send (send target-world :copy-worldcoords)  :draw-on :flush t :size 200)
    ;;
    ))

;; init robot
(defun init-robot ()
  (warning-message 2 "loading pr2 robot information~%")
  (load "package://pr2eus/pr2.l")
  (load "package://pr2eus/pr2-interface.l")
  (warning-message 2 "setup visualizer~%")
  (if (not (boundp '*pr2*)) (pr2))
  (setq *ri* (instance pr2-interface :init))
  (send *pr2* :reset-manip-pose))

;; init roseus
(defun init-roseus ()
  (warning-message 2 "setup roseus~%")
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "image_marker" image_view2::ImageMarker2 1)
  (ros::subscribe "board_pose" geometry_msgs::PoseStamped #'checkerboard-cb))

;;
(ros::roseus "vision-action-example3")
(init-robot)
(init-roseus)
;; main loop
(warning-message 2 "start main loop~%")
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  (x::window-main-one)
  )
(ros::exit)



