(load-library "roseus.so" '("roseus"))
(pushnew :ros *features*)

(defun ros::roseus-sigint-handler (sig code)
  (ros::ROS-WARN (format nil "ros::roseus-sigint-handler ~A" sig))
  (exit))

(when (not (unix:isatty *standard-input*))
  (ros::ROS-INFO "install ros::roseus-sigint-handler")
  (unix:signal unix::sigint 'ros::roseus-sigint-handler)
  (unix:signal unix::sighup 'ros::roseus-sigint-handler)
  (defmacro do-until-key (&rest forms) `(while t ,@forms))
  (defun y-or-n-p (&rest args) t))

(defun roseus (name &key
                    (option (list ros::*no-sigint-handler* ros::*anonymous-name*))
                    (args lisp::*eustop-argument*))
  (roseus-raw name (apply #'logior option) args))

(setq ros::*compile-message* nil)

(defclass ros::object
  :super propertied-object
  :slots ())
(defmethod ros::object
  (:init () self)
  (:md5sum- () (get (class self) :md5sum-))
  (:datatype- ()  (get (class self) :datatype-))
  )

(defclass ros::rostime
  :super propertied-object
  :slots (sec nsec))
(defmethod ros::rostime
  (:init (&key ((:sec _sec) 0) ((:nsec _nsec) 0))
   (ros::ros-error "ros::rostime is no longer supported")
   (setq sec _sec nsec _nsec) self)
  (:sec (&optional s) (if s (setq sec s)) sec)
  (:nsec (&optional s) (if s (setq nsec s)) nsec)
  )

(defclass ros::time
  :super propertied-object
  :slots (sec-nsec))
(defmethod ros::time
  (:init (&key ((:sec _sec) 0) ((:nsec _nsec) 0)) (setq sec-nsec (integer-vector _sec _nsec)) self)
  (:sec  (&optional s) (if s (setf (elt sec-nsec 0) s)) (elt sec-nsec 0))
  (:nsec (&optional s) (if s (setf (elt sec-nsec 1) s)) (elt sec-nsec 1))
  (:sec-nsec () sec-nsec)
  (:now () (setq sec-nsec (ros::time-now-raw)) self)
  )

(defun ros::time () (instance ros::time :init))
(defun ros::time-now () (send (instance ros::time :init) :now))

(defun roseus-add-files (pkg type)
  (let (dir dirfile)
    (setq dir (format nil "~A/~A/eus/~A/" (ros::rospack-find pkg) type pkg))
    (unless (probe-file dir)
      (warning-message 1 (format nil ";; can not find ~A directory for [~A] packages~%" dir pkg))
      (exit))
    (unless (find-package (string-upcase pkg))
      (make-package (string-upcase pkg)))
    (dolist (file (directory dir))
      (when (and (string= (pathname-type file) "l")
		 (not (= #\. (elt (pathname-name file) 0))))
	(setq dirfile (concatenate string dir file))
        (if ros::*compile-message*
            (let* ((fname (format nil "~A" (pathname-name file)))
                   (old-module (find fname *loaded-modules*
                                     :key #'lisp::load-module-file-name :test #'equal)))
              (if old-module (sys::unbinload old-module))
              (let ((pwd (unix::pwd)) lname)
                (unix::chdir (string-right-trim "/" dir))
                (comp:compile-file-if-src-newer file)
                (setq lname (load fname))
                ;;(warning-message 6 ";; loading   ~A~%" lname)
                (unix::chdir pwd)))
          (progn
            (warning-message 6 ";; loading ~A~%" dirfile)
            (load dirfile)))))
    ))

(defun roseus-add-msgs (pkg) (roseus-add-files pkg "msg"))
(defun roseus-add-srvs (pkg) (roseus-add-files pkg "srv"))

(defun ros::append-name-space (&rest args)
  (subseq (apply #'concatenate string
                 (mapcan #'(lambda (x) (list "/" x)) args)) 1)) ;remove /

(defmacro defrosparam (sym param-name &optional (default-value nil))
  ;;  defrosparam takes 3 arguments:
  ;;    1. symbol :: to bind
  ;;    2. param-name :: rosparameter name
  ;;    3. default-value :: default value when ros::get-param returns nil
  `(progn
     (if (boundp ',sym)
         (warn ";; re-definition ~A" ',sym))
     (setq ,sym (or (ros::get-param ,param-name) ,default-value))
     ',sym))

(defmacro defrosparams (&rest param-specs)
  ;; bind some rosparameters using defrosparam.
  ;; example:
  ;;  (defrosparams (*hoge* "hoge" 0.0)
  ;;                (*fuga* "/piyo/fuga" 0.0))
  `(progn
    ,@(mapcar #'(lambda (param)
                  `(defrosparam ,@param))
              param-specs)
    t)
  )

(defmacro rosparam-bind (params &rest args)
  ;;  this macro, locally bind rosparams like `let' syntax.
  ;; NB: This macro resolve parameter when macro expanding. It should 
  ;; resolve parameters in evaluation time
  ;; params := (param)
  ;; param := (symbol parameter-name)
  `(let ,(mapcar #'(lambda (param)
                     (list (car param)  ;symbol
                           (ros::get-param (cadr param))))
                 params)
     ,@args))

(defun ros-message-destructuring-bind-parse-arg (msg params)
  "this function returns a list like ((symbol value) (symbol value) ...).
always the rank of list is 2"
  (let ((notflatten-list
         (mapcar #'(lambda (param)
                     (_ros-message-destructuring-bind-parse-arg msg param))
                 params)))
    (ros-message-destructuring-bind-flatten-param notflatten-list)))

(defun ros-message-destructuring-bind-flatten-param (params)
  ;; we just check car, because cdr has list like (send msg :foobar)
  ;; params like (((hoge (send msg :hoge)) (fuga (send msg :fuga))) (piyo (send msg :piyo)))
;;  (format t "~A~%" params)
  (cond
   ((null params) nil)
   ((symbolp (car params))              ;params = (hoge (send msg :hoge))
    (list params))
   ((not (symbolp (car (car params))))  ;params != ((hoge (send msg :hoge)) ...)
    (append (ros-message-destructuring-bind-flatten-param (car params))
            (ros-message-destructuring-bind-flatten-param (cdr params))))
   ((symbolp (car (car params)))
    (append (list (car params))
            (ros-message-destructuring-bind-flatten-param (cdr params))))
   ))

(defun _ros-message-destructuring-bind-parse-arg (msg param)
  "this function returns a list like ((symbol value) ((symbol value) ...))"
  (if (listp (car param))       ;like ((:frame_id frame_id) (:stamp stamp))
      (mapcar #'(lambda (x)
                  (_ros-message-destructuring-bind-parse-arg msg x))
              param)
    (progn
      (let ((method-name (car param))
            (method-arg (cadr param)))
        (if (symbolp method-arg)
            (list method-arg `(send ,msg ,method-name))
          (_ros-message-destructuring-bind-parse-arg `(send ,msg ,method-name)
                                                     (cadr param)))))))

(defmacro ros-message-destructuring-bind (params msg &rest body)
  "usage:
 (ros-message-destructuring-bind
  ((:header ((:frame_id frame_id) (:stamp stamp))) (:pose pose))
  pose-stamped                          ;symbol to message is bound
  (print pose))"
  `(let ,(ros-message-destructuring-bind-parse-arg msg params)
     ,@body))

(setq sys::*exit-hook* 'ros::exit)
