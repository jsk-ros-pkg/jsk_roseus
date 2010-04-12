(load-library "roseus.so" '("roseus"))
(pushnew :ros *features*)

(defun ros::roseus-sigint-handler (sig code)
   (warning-message 1 "ros::interrupt")
   (ros::exit)
   (exit))

(when (not (unix:isatty *standard-input*))
  (unix:signal unix::sighup 'ros::roseus-sigint-handler 2))

(setq ros::*compile-message* nil)

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
  (:init (&key ((:sec _sec) 0) ((:nsec _nsec) 0)) (setq sec _sec nsec _nsec) self)
  (:sec (&optional s) (if s (setq sec s)) sec)
  (:nsec (&optional s) (if s (setq nsec s)) nsec)
  )
(defun ros::rostime ()
  (let ((now (ros::time-now)))
    (instance ros::rostime :init :sec (elt now 0) :nsec (elt now 1))))

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
        (if ros::*compile-message*
            (let* ((fname (format nil "~A" (pathname-name file)))
                   (old-module (find fname *loaded-modules*
                                     :key #'lisp::load-module-file-name :test #'equal)))
              (if old-module (sys::unbinload old-module))
              (let ((pwd (unix::pwd)) lname)
                (unix::cd (string-right-trim "/" dir))
                (comp:compile-file-if-src-newer file)
                (setq lname (load fname))
                (warning-message 6 ";; loading   ~A~%" lname)
                (unix::cd pwd)))
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

(setq sys::*exit-hook* 'ros::exit)
