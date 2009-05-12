(load "roseus.l")

(roseus-add-msgs "roseus")
(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "robot_msgs")
;;;
(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::advertise "clientserver" robot_msgs::PositionMeasurement 1)
(dotimes (i 10)
  (setq msg (instance robot_msgs::PositionMeasurement :init))
  (send msg :name (format nil "name ~d" i))
  (send msg :object_id (format nil "objectid ~d" i))
  (send msg :pos :x (+ 10 (* i 0.1)))
  (send msg :pos :y (+ 11 (* i 0.1)))
  (send msg :pos :z (+ 12 (* i 0.1)))
  (send msg :reliability (+ 100 (* i 0.1)))
  (setf (elt (send msg :covariance) 0) (+ 200 (* i 0.1)))
  (setf (elt (send msg :covariance) 1) (+ 201 (* i 0.1)))
  ;;(send msg :pos (format nil "objectid ~d" i)))
  (ros::publish "clientserver" msg)
  (unix:usleep (* 200 1000))
  )
(roseus "shutdown")
(exit)

