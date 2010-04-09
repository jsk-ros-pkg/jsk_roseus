(load-library "roseus.so" '("roseus"))
(pushnew :ros *features*)

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

(setq sys::*exit-hook* 'ros::exit)

