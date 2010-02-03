(pushnew (format nil "~A/euslisp/"
                 (read (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
;;;

(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
;;(defun test2 (a) (print (cons a "test")) 1)
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))
(ros::subscribe "chatter" roseus::string #'(lambda (msg) (print (send msg :data))))

(ros::init-ros-wakeup-worker-thread)







