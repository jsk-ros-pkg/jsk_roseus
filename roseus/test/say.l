#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "sound_play")
;;;
(setq ros::*debug* t)
;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A ~A~%" (sys::thread-self) a b)))
(ros::roseus "clear")

(ros::advertise "robotsound" sound_play::SoundRequest 5)
(unix:sleep 1) ;; this is VERY important

(setq msg (instance sound_play::SoundRequest :init))
;;
(send msg :sound sound_play::SoundRequest::*say*) ;; say
(send msg :command sound_play::SoundRequest::*play_once*) ;; once
;; since sound_play version 0.3.1, the message including volume slot. c.f. https://github.com/ros-drivers/audio_common/pull/51
(when (find-method msg :volume)
  (send msg :volume 10))
(send msg :arg "Hello")

(ros::publish "robotsound" msg)
(read-char)

(ros::roseus "shutdown")
(exit)




