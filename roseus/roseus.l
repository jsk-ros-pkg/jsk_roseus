;;	   (boundp 'ros::*roseus-walkup-thread*))
;  (print "akasii")
;  (setq ros::*roseus-walkup-thread-loop* nil)
;  (sys::wait-thread ros::*roseus-walkup-thread*)
;  (print ros::*roseus-walkup-thread*))

(load-library "lib/libroseus.so" '("roseus"))
(when (and (boundp 'ros::*roseus-walkup-thread*)
	   ros::*roseus-walkup-thread*)
  (ros::roseus-exit))

(defclass ros::object
  :super propertied-object
  :slots ())
(defmethod ros::object
  (:init () self)
  (:md5sum () (get (class self) :md5sum))
  (:type ()  (get (class self) :type))
  )

(defclass ros::rostime
  :super propertied-object
  :slots (sec nsec))
(defmethod ros::rostime
  (:init (&optional (_sec 0) (_nsec 0)) (setq sec _sec nsec _nsec) self)
  (:sec (&optional s) (if s (setq sec s)) sec)
  (:nsec (&optional s) (if s (setq nsec s)) nsec)
  )

(defun roseus-add-msgs (pkg)
  (let (dir dirfile tmpfile)
    ;;(setq dir (format nil "~A/msg/eus/~A/" (read-line (piped-fork (format nil "rospack find ~A" pkg))) pkg))
    (setq tmpfile (format nil "/tmp/roseus-rospack-find-~A.tmp" (unix::getpid)))
    (unix::system (format nil "rospack find ~A > ~A" pkg tmpfile))
    (setq dir (format nil "~A/msg/eus/~A/" (with-open-file (f tmpfile :direction :input) (read-line f)) pkg))
    (unix::unlink tmpfile)
    (unless (probe-file dir)
      (warning-message 1 (format nil ";; can not find ~A directory for [~A] packages~%" dir pkg))
      (exit))
    (unless (find-package (string-upcase pkg))
      (make-package (string-upcase pkg)))
    (dolist (file (directory dir))
      (when (and (string= (pathname-type file) "l")
		 (not (= #\. (elt (pathname-name file) 0))))
	(setq dirfile (concatenate string dir file))
	(warning-message 6 ";; loading ~A~%" dirfile)
	(load dirfile)))))

;;;
;;;

(defun ros::advertise (topic msg-class &optional (queuesize 1))
  (let ((md5sum (get msg-class :md5sum))
	(type (get msg-class :type)))
    (when ros::*debug*
      (warn "ros::advertise ~A ~A~%" topic type))
    (unless (and (stringp md5sum) (stringp type))
      (error "ros::advertise md5sum(~A) and type(~A) of ~A must be string~%"
	     md5sum type msg-class))
    (ros::roseus_advertise topic md5sum type queuesize)))

(defun ros::publish (topic msg)
  (let (buf buflen)
    (setq buf (send msg :serialize)
	  buflen (send msg :serialization-length))
    (when ros::*debug*
      (let (c)
	(warn "ros::publish ~A [~d] " topic buflen)
	(dotimes (i buflen)
	  (setq c (elt buf i))
	  (if (alphanumericp c)	(warn "~c" c) (warn "~0,2x" c)))
	(warn "(~d)~%" (length buf))))
    (ros::roseus_publish topic buf buflen)))

(defun ros::subscribe (topic msg-class func &optional (queuesize 1))
  (let ((md5sum (get msg-class :md5sum))
	(type (get msg-class :type))
	subscribe-func)
    (when ros::*debug*
      (warn "ros::subscribe ~A ~A~%" topic type))
    (setq subscribe-func
     (eval `(function
	     (lambda (s)
	       (let ((msg (instance ,(send msg-class :name) :init)))
		 (send msg :deserialize s)
		 (funcall ',func msg)
		 )))))
    (if (boundp 'ros::*roseus-walkup-thread*)
	(push subscribe-func ros::*roseus-walkup-thread-funcs*))
    (ros::roseus_subscribe topic md5sum type subscribe-func queuesize)
    ))


;;;
;;;
(defun ros::roseus-exit ()
  (when (boundp 'ros::*roseus-walkup-thread*)
    (setq ros::*roseus-walkup-thread-loop* nil)
    (sys::wait-thread ros::*roseus-walkup-thread*)
    (setq ros::*roseus-walkup-thread-funcs* nil)
    (setq ros::*roseus-walkup-thread* nil))
  (ros::roseus_exit))
(setq sys::*exit-hook* 'ros::roseus-exit)
(defun ros::roseus_wakeup_worker () (ros::roseus_worker))
;;
(defun ros::init-ros-wakeup-worker-itimer ()
  (unless lisp::*use-top-selector*
    (error "roseus requires TOP-SELECTOR~%"))
  (when lisp::*use-top-selector*
    (setq lisp::*top-selector-interval* 0.1)
    (unless (member 'ros::roseus_wakeup_worker lisp::*timer-job*)
      (push 'ros::roseus_wakeup_worker lisp::*timer-job*))))
(defun ros::init-ros-wakeup-worker-thread ()
  (when (< (length (sys::free-threads)) 1) (sys::make-thread 1))
  (unless (and (boundp 'ros::*roseus-walkup-thread*)
	       'ros::*roseus-walkup-thread*)
    (setq ros::*roseus-walkup-thread-funcs* nil)
    (setq ros::*roseus-walkup-thread-loop* t)
    (setq ros::*roseus-walkup-thread*
	  (sys::thread #'(lambda ()
			   (while ros::*roseus-walkup-thread-loop*
			     (ros::roseus_worker) (unix:usleep (* 100 1000)))
			   )))))

;;(ros::init-ros-wakeup-worker-itimer)
;;(ros::init-ros-wakeup-worker-thread)



