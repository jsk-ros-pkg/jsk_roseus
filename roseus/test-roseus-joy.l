(require :orbixif (format nil "~A/hrp2/corba/orbixif.l" (unix:getenv "CVSDIR")))
(require :plugins (format nil "~A/hrp2/plugins/plugins.l" (unix:getenv "CVSDIR")))
(need-thread 6)

(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "robot_msgs")
(roseus-add-msgs "joy")
;;;
(setq ros::*debug* t)
;;;

(unless (boundp '*hrp2*) (hrp2v-simple))
(unless (boundp '*ci*) (eusorbix-init))

;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A ~A~%" (sys::thread-self) a b)))
(ros::init-ros-wakeup-worker-thread)
(roseus "clear")

(defun test (msg)
  (let  (axes buttons)
    (setq axes (send msg :axes) 
	  buttons (send msg :buttons))
    (print (list (sys::thread-self) msg))
    (print (list axes buttons))
    (when (> (length buttons) 0)
      (if (eq (elt buttons 3) 1)
	  (send *ci* :go-velocity (* (elt axes 2) -0.1) (* (elt axes 3) 0.1) (* (elt axes 0) 10.0))
      (send *ci* :go-velocity 0 0 0)))
    ))
(ros::subscribe "joy" joy::Joy #'test 10)




