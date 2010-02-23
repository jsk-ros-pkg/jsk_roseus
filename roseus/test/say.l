(pushnew (format nil "~A/euslisp/" (read-line (unix::piped-fork "rospack find roseus"))) *load-path*)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "sound_play")
;;;
(setq ros::*debug* t)
;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A ~A~%" (sys::thread-self) a b)))
(roseus "clear")

(ros::advertise "robotsound" sound_play::SoundRequest 5)
(unix:sleep 1) ;; this is VERY important

(setq msg (instance sound_play::SoundRequest :init))
;;
(send msg :sound sound_play::say) ;; say
(send msg :command sound_play::play_once) ;; once
(send msg :arg "Hello")

(ros::publish "robotsound" msg)
(read-char)

(roseus "shutdown")
(exit)




