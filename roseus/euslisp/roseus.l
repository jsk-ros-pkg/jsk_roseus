;;	   (boundp 'ros::*roseus-walkup-thread*))
;  (print "akasii")
;  (setq ros::*roseus-walkup-thread-loop* nil)
;  (sys::wait-thread ros::*roseus-walkup-thread*)
;  (print ros::*roseus-walkup-thread*))

(load-library "roseus.so" '("roseus"))
(when (and (boundp 'ros::*roseus-walkup-thread*)
	   ros::*roseus-walkup-thread*)
  (ros::roseus-exit))

(if (not (boundp 'ros::*debug*)) (setq ros::*debug* nil))

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

(defun roseus-add-files (pkg type)
  (let (dir dirfile)
    (setq dir (format nil "~A/~A/eus/~A/"
                      (read (piped-fork (format nil "rospack find ~A" pkg)))
                      type pkg))
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

(defun roseus-add-msgs (pkg) (roseus-add-files pkg "msg"))
(defun roseus-add-srvs (pkg) (roseus-add-files pkg "srv"))

;;;
;;;
(defun ros::print-buf (name type buf buflen)
  (let (c)
    (warn "~A ~A [~d] " type name buflen)
    (dotimes (i buflen)
      (setq c (elt buf i))
      (if (alphanumericp c)	(warn "~c" c) (warn "~0,2x" c)))
    (warn "(~d)~%" (length buf))))
;;;

(defun ros::advertise (topic msg-class &optional (queuesize 1))
  (let ((md5sum (get msg-class :md5sum))
	(type (get msg-class :type))
        (definition (or (get msg-class :definition) "")))
    (when ros::*debug*
      (warn "ros::advertise ~A ~A~%" topic type))
    (unless (and (stringp md5sum) (stringp type) (stringp definition))
      (error "ros::advertise md5sum(~A), type(~A) and definition(~A) of ~A must be string~%"
	     md5sum type definition msg-class))
    (ros::roseus_advertise topic md5sum type definition queuesize)))

(defun ros::unadvertise (topic)
  (ros::roseus_advertise topic))

(defun ros::publish (topic msg)
  (let ((md5sum (get (class msg) :md5sum))
        (type (get (class msg) :type))
        (buf (send msg :serialize))
        (buflen (send msg :serialization-length)))
    (when ros::*debug* (ros::print-buf "ros::publish" topic buf buflen))
    (ros::roseus_publish topic md5sum buf buflen type)))

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
    (setq *subscribe-func* subscribe-func *ros-subscribe-func* func)
    (if (boundp 'ros::*roseus-walkup-thread*)
	(push subscribe-func ros::*roseus-walkup-thread-funcs*))
    (ros::roseus_subscribe topic md5sum type subscribe-func queuesize)
    ))

(defun ros::wait-for-service (service)
  (ros::roseus_wait_for_service service))

(defun ros::service-call (service msg)
  (let ((md5sum (get (class msg) :md5sum))
        (type (get (class msg) :type))
        (req-buf (send msg :request :serialize))
        (req-buflen (send msg :request :serialization-length))
        (res-buf (send msg :response :serialize))
        (res-buflen (send msg :response :serialization-length)))
    (when ros::*debug* (ros::print-buf "ros::service-call->request" service req-buf req-buflen))
    (ros::roseus_service_call service md5sum req-buf req-buflen res-buf res-buflen type)
    (when ros::*debug* (ros::print-buf "ros::service-call->response" service res-buf res-buflen))
    (send msg :response :deserialize res-buf)
    msg))

(defun ros::advertise-service (service msg-class func)
  (let ((md5sum (get msg-class :md5sum))
	(req-type (get msg-class :response-type))
	(res-type (get msg-class :request-type))
	(type (get msg-class :type))
	service-func)
    (when ros::*debug*
      (warn "ros::advertise-service ~A ~A~%" service type))
    (setq advertise-func
     (eval `(function
	     (lambda (s)
	       (let ((msg (instance ,(send msg-class :name) :init)))
		 (send msg :request :deserialize s)
		 (funcall ',func msg)
                 (send msg :response :serialize))))))
    (setq *advertise-func* advertise-func *ros-service-func* func)
    (if (boundp 'ros::*roseus-walkup-thread*)
	(push subscribe-func ros::*roseus-walkup-thread-funcs*))
    (ros::roseus_advertise_service service md5sum req-type res-type type advertise-func)
    ))

(defun ros::unadvertise-service (service)
  (ros::roseus_advertise-service service))
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

;; rosconsole api
(defun ros::info (str &rest args)
  (roseus_rosinfo (apply #'format nil str args)))
(defun ros::warn (str &rest args)
  (roseus_roswarn (apply #'format nil str args)))
(defun ros::error (str &rest args)
  (roseus_roserror (apply #'format nil str args)))
(defun ros::fatal (str &rest args)
  (roseus_rosfatal (apply #'format nil str args)))
;;(ros::init-ros-wakeup-worker-itimer)
;;(ros::init-ros-wakeup-worker-thread)



