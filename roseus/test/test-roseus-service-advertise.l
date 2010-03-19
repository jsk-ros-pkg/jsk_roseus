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
(setq *add-two-ints-lock* (sys::make-mutex-lock))
(setq *data* nil)
(defun add-two-ints (msg)
  (send msg :response :sum
        (+ (send msg :request :a)
           (send msg :request :b)))
  (print (list (sys::thread-self) (send msg :response :sum)))
  )
;;;
;;;
(roseus "clear")
(ros::advertise-service "add_two_ints" rospy_tutorials::AddTwoInts #'add-two-ints)
(ros::init-ros-wakeup-worker-thread)

(do-until-key
 (print *data*)
 (unix:usleep 100000))



