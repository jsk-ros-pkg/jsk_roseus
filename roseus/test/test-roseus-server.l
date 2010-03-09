(pushnew (format nil "~A/euslisp/" (read-line (unix::piped-fork "rospack find roseus"))) *load-path*)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")

;;;
(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::advertise "clientserver" roseus::String 1)
(dotimes (i 100)
  (setq msg (instance roseus::String :init))
  (send msg :data (format nil "(scale ~A (float-vector 1 2 3 4 5)" i))
  (ros::publish "clientserver" msg)
  (unix:usleep (* 200 1000))
  )
(roseus "shutdown")
(exit)

