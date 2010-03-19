;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;

(pushnew (format nil "~A/euslisp/"
                 (read (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-srvs "rospy_tutorials")
;;;

(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::wait-for-service "add_two_ints")
(setq tmp (instance rospy_tutorials::AddTwoInts :init))
(send tmp :request :a 10)
(send tmp :request :b 20)
(setq tmp (ros::service-call "add_two_ints" tmp))
(format t "sum = ~A~%" (send tmp :response :sum))


