(when (not (find "roseus"
                 lisp::*loaded-modules*
                 :key #'lisp::load-module-file-name
                 :test #'equal))
  (make-package "ROS")
  (load-library "roseus.so" '("roseus")))

(unless (member :ros *features*) (push :ros *features*))
(warning-message 1 ";; loading roseus(~A) on euslisp(~A)~%" (car ros::roseus-svnversion) (cadr lisp::lisp-implementation-version))

(in-package "ROS")

(defun ros::roseus-sigint-handler (sig code)
  (ros::ROS-WARN (format nil "ros::roseus-sigint-handler ~A" sig))
  (exit 1))

(defun ros::roseus-error (code msg1 form &optional (msg2))
   (format *error-output* "~C[1;3~Cm~A roseus-error: ~A"
                #x1b (+ 1 48)   *program-name* msg1)
   (if msg2 (format *error-output* " ~A" msg2))
   (if form (format *error-output* " in ~s" form))
   (format *error-output* ", exitting...~C[0m~%"  #x1b)
   (exit 1))

(when (not (unix:isatty *standard-input*))
  (ros::ROS-INFO "install ros::roseus-sigint-handler")
  (lisp::install-error-handler 'ros::roseus-error)
  (unix:signal unix::sigint 'ros::roseus-sigint-handler)
  (unix:signal unix::sighup 'ros::roseus-sigint-handler)
  (defmacro user::do-until-key (&rest forms) `(while t ,@forms))
  (defun user::y-or-n-p (&rest args) t))

(defun ros::roseus (name &key
                    (option (list ros::*no-sigint-handler* ros::*anonymous-name*))
                    (args lisp::*eustop-argument*))
  (ros::roseus-raw name (apply #'logior option) args))

(setq ros::*compile-message* nil)

(shadow 'object *package*)
(defclass ros::object
  :super propertied-object
  :slots ())
(defmethod ros::object
  (:init () self)
  (:md5sum- () (get (class self) :md5sum-))
  (:datatype- ()  (get (class self) :datatype-))
  )

(defclass ros::time
  :super ros::object
  :slots (sec-nsec))
(defmethod ros::time
  (:init (&key ((:sec _sec) 0) ((:nsec _nsec) 0))
         (setq sec-nsec (integer-vector _sec _nsec)) self)
  (:sec  (&optional s) (if s (setf (elt sec-nsec 0) s)) (elt sec-nsec 0))
  (:nsec (&optional s) (if s (setf (elt sec-nsec 1) s)) (elt sec-nsec 1))
  (:sec-nsec () sec-nsec)
  (:now () (setq sec-nsec (ros::time-now-raw)) self)
  (:to-sec () (+ (send self :sec) (/ (send self :nsec) 1000000000.0)))
  (:to-nsec () (+ (* (send self :sec) 1000000000.0) (send self :nsec)))
  (:from-sec (sec)
             (send self :sec (floor sec))
             (send self :nsec (floor (* 1000000000 (- sec (floor sec)))))
             self)
  (:from-nsec (nsec)
              ;; divide by 1000 to avoid  big div ?
              (send self :sec  (/ (/ (floor nsec) 1000) 1000000))
              (send self :nsec (* 1000 (mod (/ (floor nsec) 1000) 1000000)))
              self)
  (:prin1 (&optional (strm t) &rest msgs)
	  (let* ((ns (+ (* 1000000000 (send self :sec)) (send self :nsec)))
		 (str (format nil (if (< 0 ns) "~0,10d" "~0,11d") ns))
		 (dpt (- (length str) 9)))
	    (send-super :prin1 strm
			(format nil "~A.~A" (subseq str 0 dpt)
				(subseq str dpt (+ 3 dpt)))))) ;; print .xxx
  )

(defun ros::time (&optional (sec 0))
  (send (instance ros::time :init) :from-sec sec))

(defun ros::time-now () (send (instance ros::time :init) :now))

(defun ros::time+ (a b)
  (let* ((csec  (+ (send a :sec)  (send b :sec)))
	 (cnsec (+ (send a :nsec) (send b :nsec))))
    (if (>= cnsec 1000000000) (setq csec (+ csec 1)
				    cnsec (- cnsec 1000000000)))
    (instance ros::time :init :sec csec :nsec cnsec)))
(defun ros::time- (a b)
  (let* ((csec  (- (send a :sec)  (send b :sec)))
	 (cnsec (- (send a :nsec) (send b :nsec))))
    (if (< cnsec 0) (setq csec (- csec 1) cnsec (+ cnsec 1000000000)))
    (instance ros::time :init :sec csec :nsec cnsec)))

(defun ros::roseus-add-files (pkg type)
  (let ((dir (format nil "~A/~A/eus/~A/" (ros::rospack-find pkg) type pkg)))
    (unless (probe-file dir)
      (warning-message
       1 (format nil ";; can not find ~A directory for [~A] packages~%" dir pkg))
      (exit 1))
    (unless (find-package (string-upcase pkg))
      (make-package (string-upcase pkg)))
    (dolist (file (directory dir))
      (when (and (string= (pathname-type file) "l")
		 (not (= #\. (elt (pathname-name file) 0))))
	(let ((dirfile (concatenate string dir file))
              (modname (intern (string-upcase (format nil "~A/~A" pkg (pathname-name file))) *keyword-package*)))
          (if ros::*compile-message*
              (let* ((fname (pathname-name file))
                     (old-module (find fname *loaded-modules*
                                       :key #'lisp::load-module-file-name
                                       :test #'equal)))
                (if old-module (sys::unbinload old-module))
                (let ((pwd (unix::pwd)))
                  (unix::chdir (string-right-trim "/" dir))
                  (comp:compile-file-if-src-newer file)
                  (load fname)
                  (unix::chdir pwd)))
            (require modname dirfile)))))))

(defun ros::roseus-add-msgs (pkg) (ros::roseus-add-files pkg "msg"))
(defun ros::roseus-add-srvs (pkg) (ros::roseus-add-files pkg "srv"))

(unless (fboundp 'load-org-for-ros)
  (setf (symbol-function 'load-org-for-ros) (symbol-function 'load)))
(defun ros::resolve-ros-path (fname)
  (let* ((urlname (url-pathname fname))
	 (package-name (send urlname :host))
	 (path-name (namestring urlname))
	 (package-path (ros::rospack-find package-name)))
    (if package-path
	(format nil "~A/~A" package-path path-name)
      (progn (warning-message 1 ";; counldn't find pacakge [~a]~%" package-name) nil))))

(defun load (fname &rest args)
  (if (substringp "package://" fname)
    (setq fname (ros::resolve-ros-path fname)))
  (if fname (apply #'load-org-for-ros fname args)))

;;; defmacro for rosparams

(defun ros::append-name-space (&rest args)
  (subseq (apply #'concatenate string
                 (mapcan #'(lambda (x) (list "/" x)) args)) 1)) ;remove /

(defmacro defrosparam (sym param-name &optional (default-value nil))
  ;; defrosparam bind a ros parameter to lisp symbol using setq, not defvar.
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

(defun ros-slot-ref (inst slot)
  (send inst slot))

(defsetf ros-slot-ref (inst slot) (val)
  `(send ,inst ,slot ,val))

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
            (list method-arg `(ros-slot-ref ,msg ,method-name))
          (_ros-message-destructuring-bind-parse-arg
           `(ros-slot-ref ,msg ,method-name)
           (cadr param)))))))

(defmacro ros-message-destructuring-bind (params msg &rest forms)
;;  usage:
;; (ros-message-destructuring-bind
;;  ((:header ((:frame_id frame_id) (:stamp stamp))) (:pose pose))
;;  pose-stamped                          ;symbol to message is bound
;;  (print pose))
  `(symbol-macrolet ,(ros-message-destructuring-bind-parse-arg msg params)
     ,@forms))

(defun ros::load-ros-manifest (pkg)
  (let ((manifest (format nil "~A/src/euslisp/_manifest.l"
                          (ros::rospack-find pkg))))
    (when (probe-file manifest)
      (load-org-for-ros manifest))))

(setq sys::*exit-hook* 'ros::exit)

(provide :roseus "roseus.l")

;; obsoleted functions
(in-package "USER")
(defun roseus (&rest args)
  (warning-message 1 ";; roseus is obsoleted, use ros::roseus~%")
  (apply #'ros::roseus args))
(defun getname ()
  (warning-message 1 ";; getname is obsoleted, use ros::get-name~%")
  (format t "~C[3~Cm" #x1b 49)
  (btrace)
  (format t "~C[0m" #x1b)
  (ros::get-name))
(defun roseus-add-msgs (pkg)
  (warning-message 3 ";; roseus-add-msgs will be obsoleted, use ros::roseus-add-msgs [~s]~%" pkg)
  (ros::roseus-add-msgs pkg))
(defun roseus-add-srvs (pkg)
  (warning-message 3 ";; roseus-add-srvs will be obsoleted, use ros::roseus-add-srvs [~s]~%" pkg)
  (ros::roseus-add-srvs pkg))
(defun load-ros-manifest (pkg)
  (warning-message 3 ";; load-ros-manifest will be obsoleted, use ros::load-ros-manifest [~s]~%" pkg)
  (ros::load-ros-manifest pkg))
