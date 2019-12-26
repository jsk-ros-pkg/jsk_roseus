#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "sound_play")
(ros::load-ros-manifest "face_detector_mono")
;;;
;;;

;; vision callback
(defun facedetect-cb (faces)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) ret)
    (warning-message 2 "facedetect callback~%")
    (when (send faces :rects)
      (let* ((f (car (send faces :rects)))
             (cx (send f :x)) (cy (send f :y))
             (w  (send f :width))  (w/2 (/ w 2))
             (h  (send f :height)) (h/2 (/ h 2))
	     (msg (instance sound_play::SoundRequest :init))
	     cam p v)
        (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)

        (send mrk :type image_view2::ImageMarker2::*POLYGON*)
        (send mrk :points (list
                           (instance geometry_msgs::Point :init
                                     :x (- cx w/2) :y (- cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (- cx w/2) :y (+ cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (+ cx w/2) :y (+ cy h/2))
                           (instance geometry_msgs::Point :init
                                     :x (+ cx w/2) :y (- cy h/2))))
        (ros::publish "image_marker" mrk)
	;;
	;; greetings
	(send msg :sound sound_play::SoundRequest::*say*) ;; say
	(send msg :command sound_play::SoundRequest::*play_once*) ;; once
        (when (fid-method msg :volume)
          (send msg :volume 1.0))
	(send msg :arg "こんにちは")
	(ros::publish "robotsound" msg)
	;;
	;; calc 3d ray from camera model and image coordinates
	(setq cam (send *pr2* :camera "wide_stereo/right"))
	(setq p (send cam :worldpos))
	(setq v (send cam :ray cx cy))
	(format t "ray = ~A~%" v)

	;; look-at the end of ray
	(send *pr2* :head :look-at (v+ p (scale 1000 v)))
	(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
	(send *viewer* :viewsurface :3d-line p (v+ p (scale 1000 v)))
	(send *viewer* :viewsurface :flush)
	;;
	))
    ))


;; init robot
(defun init-robot ()
  (warning-message 2 "loading pr2 robot information~%")
  (load "package://pr2eus/pr2.l")
  (load "package://pr2eus/pr2-interface.l")
  (warning-message 2 "setup visualizer~%")
  (if (not (boundp '*pr2*)) (pr2))
  (setq *ri* (instance pr2-interface :init))
  (send *pr2* :reset-pose))

;; init roseus
(defun init-roseus ()
  (warning-message 2 "setup roseus~%")
  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "image_marker" image_view2::ImageMarker2 1)
  (ros::subscribe "face_detector_mono/faces" face_detector_mono::RectArray #'facedetect-cb))

;;
(ros::roseus "vision-action-example2")
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



