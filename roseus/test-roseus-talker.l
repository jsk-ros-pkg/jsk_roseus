(load "roseus.l")

(roseus-add-msgs "roseus")
;;;

(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::advertise "chatter" roseus::string 1)
(dotimes (i 100)
  (setq msg (instance roseus::string :init))
  (send msg :data (format nil "hello world ~d" (+ 95 i)))
  (ros::publish "chatter" msg)
  (print (send msg :data))
  (unix:usleep (* 200 1000))
  )
(roseus "shutdown")
(exit)

