(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")
;;;
(setq ros::*debug* t)
;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A ~A~%" (sys::thread-self) a b)))
(roseus "clear")
(defun test (msg)
  (dotimes (i 100) (instance std_msgs::ColorRGBA :init))
  (print (list (sys::thread-self) msg))
  )
(ros::subscribe "clientserver" std_msgs::ColorRGBA #'test 10)




