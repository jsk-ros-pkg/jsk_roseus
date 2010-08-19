#!/usr/bin/env runeus

(load "roseus.l")

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "sound_play")
;;;
(setq ros::*debug* t)
;;;
;;;
(roseus "jtalk")

(ros::advertise "robotsound" sound_play::SoundRequest 5)
(unix:sleep 1) ;; this is VERY important

(setq msg (instance sound_play::SoundRequest :init))
;;
(send msg :sound sound_play::say) ;; say
(send msg :command sound_play::play_once) ;; once
(send msg :arg "おはようございます．きょうはいい天気ですか？")

(ros::publish "robotsound" msg)
(unix:sleep 5)

(roseus "shutdown")
(exit)
