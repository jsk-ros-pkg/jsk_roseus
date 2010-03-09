(pushnew (format nil "~A/euslisp/" (read-line (unix::piped-fork "rospack find roseus"))) *load-path*)
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
(setq *stringcb-lock* (sys::make-mutex-lock))
(setq *data* nil)
(defun stringcb (msg)
  (print (list (sys::thread-self) (read-from-string (send msg :data))))
  (sys::mutex *stringcb-lock*
              (setq *data* (read-from-string (send msg :data))))
  )
(ros::subscribe "clientserver" roseus::String #'stringcb 10)
(ros::init-ros-wakeup-worker-thread)

(do-until-key
 (print *data*)
 (unix:usleep 100000))



