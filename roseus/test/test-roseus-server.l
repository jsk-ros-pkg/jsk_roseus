(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")

;;;
(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::advertise "clientserver" std_msgs::ColorRGBA 1)
(dotimes (i 100)
  (setq msg (instance std_msgs::ColorRGBA :init))
  (send msg :r (+ 10 (* i 0.1)))
  (send msg :g (+ 11 (* i 0.1)))
  (send msg :b (+ 12 (* i 0.1)))
  (send msg :a (+ 100 (* i 0.1)))
  (ros::publish "clientserver" msg)
  (unix:usleep (* 200 1000))
  )
(roseus "shutdown")
(exit)

