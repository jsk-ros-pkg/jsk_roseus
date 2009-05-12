(load "roseus.l")

(roseus-add-msgs "roseus")
(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "robot_msgs")
;;;
(setq ros::*debug* t)
;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A ~A~%" (sys::thread-self) a b)))
(roseus "clear")
;;(ros::subscribe "clientserver" robot_msgs::PositionMeasurement #'(lambda (msg) (print msg)) 10)
(defun test (msg)
  (dotimes (i 100) (instance robot_msgs::PositionMeasurement :init))
  (print (list (sys::thread-self) msg))
  )
(ros::subscribe "clientserver" robot_msgs::PositionMeasurement #'test 10)




