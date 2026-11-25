(when (not (find "roseus"
                 lisp::*loaded-modules*
                 :key #'lisp::load-module-file-name
                 :test #'equal))
  (make-package "ROS")
  (unless (some #'(lambda (p) (probe-file (concatenate-pathnames  p "roseus.so")))
                (union *loader-current-directory* *load-path* :test #'string=))
    (warning-message 1 ";; could not find roseus.so, try rosmake roseus")
    (exit -1))
  (load-library "roseus.so" '("roseus")))

(unless (member :ros *features*) (push :ros *features*))
;; roseus.so of ROS 2 defines ros::*ros-version*
(when (and (boundp 'ros::*ros-version*) (= (symbol-value 'ros::*ros-version*) 2))
  (pushnew :ros2 *features*))
(warning-message 3 ";; loading roseus(~A) on euslisp(~A)~%" (car ros::roseus-repo-version) lisp::lisp-implementation-version)

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

;; check if this process is created by roslaunch
(when (let ((arg (car (last lisp::*eustop-argument*))))
        (and (>= (length arg) 7) (string= "__log:=" (subseq arg 0 7))))
  (ros::ROS-INFO "install ros::roseus-sigint-handler")
  (setq lisp::*exit-on-fatal-error* t)
  (lisp::install-error-handler 'ros::roseus-error)
  (unix:signal unix::sigint 'ros::roseus-sigint-handler)
  (unix:signal unix::sighup 'ros::roseus-sigint-handler)
  (defmacro user::do-until-key (&rest forms) `(while t ,@forms))
  (defun user::y-or-n-p (&rest args) t))

(defun ros::roseus (name &key
                    (anonymous t)
                    (logger "ros.roseus")
                    (level ros::*rosinfo*)
                    (args lisp::*eustop-argument*))
  "Register roseus client node with the master under the specified name"
  ;; (pprint (list 'roseus_args args))
  (let ((option (if anonymous (list ros::*anonymous-name*) (list))))
    (prog1
        (ros::roseus-raw name (apply #'logior option) args)
      (dotimes (i (length lisp::*eustop-argument*))
        (when (and
               (not (substringp "warning-message" (elt lisp::*eustop-argument* i)))
               (substringp ":=" (elt lisp::*eustop-argument* i)))
          (setf (elt lisp::*eustop-argument* i)
                (format nil "(warning-message 1 \"ignore ~a~%\")"
                        (elt lisp::*eustop-argument* i)))))
      (ros::set-logger-level logger level)
      )))

(setq ros::*compile-message* nil)

(shadow 'object *package*)
(defclass ros::object
  :super propertied-object
  :slots (_connection-header))
(defmethod ros::object
  (:init () self)
  (:md5sum- () (get (class self) :md5sum-))
  (:datatype- ()  (get (class self) :datatype-))
  (:connection-header () _connection-header))

(shadow 'time *package*)
(defclass ros::time
  :super ros::object
  :slots (sec-nsec))
(defmethod ros::time
  (:init (&key ((:sec _sec) 0) ((:nsec _nsec) 0))
         "ros::time represents ROS time primitive type, which consits of two integers, seconds and nanoseconds"
         (setq sec-nsec (integer-vector _sec _nsec)) self)
  (:sec  (&optional s) (if s (setf (elt sec-nsec 0) s)) (elt sec-nsec 0))
  (:nsec (&optional s) (if s (setf (elt sec-nsec 1) s)) (elt sec-nsec 1))
  (:sec-nsec () sec-nsec)
  (:now () (setq sec-nsec (ros::time-now-raw)) self)
  (:to-sec () (+ (send self :sec) (/ (send self :nsec) 1000000000.0)))
  (:to-nsec () (+ (* (send self :sec) 1000000000) (send self :nsec)))
  (:from-sec (sec)
             "create new ros::time instance from a float seconds representation"
             (send self :sec (floor sec))
             (send self :nsec (floor (* 1000000000 (- sec (floor sec)))))
             self)
  (:from-nsec (nsec)
              ;; divide by 1000 to avoid  big div ?
              (send self :sec  (/ (/ (floor nsec) 1000) 1000000))
	      ;; avoid mod bigint
              (send self :nsec (- (floor nsec) (* (elt sec-nsec 0) 1000000000)))
              self)
  (:prin1 (&optional (strm t) &rest msgs)
	  (let* ((ns (+ (* 1000000000 (send self :sec)) (send self :nsec)))
		 (str (format nil (if (<= 0 ns) "~0,10d" "~0,11d") ns))
		 (dpt (- (length str) 9)))
	    (send-super :prin1 strm
			(format nil "~A.~A" (subseq str 0 dpt)
				(subseq str dpt (+ 3 dpt)))))) ;; print .xxx
  )

(defun ros::time (&optional (sec 0))
  "Create new ros::time instance"
  (send (instance ros::time :init) :from-sec sec))

(defun ros::time-now () "Create new ros::time instance representing current time" (send (instance ros::time :init) :now))

(defun ros::time+ (a b)
  "add ros::time"
  (let* ((csec  (+ (send a :sec)  (send b :sec)))
	 (cnsec (+ (send a :nsec) (send b :nsec))))
    (if (>= cnsec 1000000000) (setq csec (+ csec 1)
				    cnsec (- cnsec 1000000000)))
    (instance ros::time :init :sec csec :nsec cnsec)))
(defun ros::time- (a b)
  "substruct ros::time"
  (let* ((csec  (- (send a :sec)  (send b :sec)))
	 (cnsec (- (send a :nsec) (send b :nsec))))
    (if (< cnsec 0) (setq csec (- csec 1) cnsec (+ cnsec 1000000000)))
    (instance ros::time :init :sec csec :nsec cnsec)))
(defun ros::time= (a b)
  "check if two ros::time is equal"
  (equal (a . sec-nsec) (b . sec-nsec)))
(defun ros::time> (a b)
  "check if given ros::time is greater then the others"
  (let ((t-dif (coerce (send (ros::time- a b) :sec-nsec) cons)))
    (or (> (car t-dif) 0) (and (= (car t-dif) 0) (> (cadr t-dif) 0)))))
(defun ros::time>= (a b)
  "check if given ros::time is greater then or equal to the others"
  (or (ros::time= a b) (ros::time> a b)))
(defun ros::time< (a b)
  "check if given ros::time is less then the others"
  (not (ros::time>= a b)))
(defun ros::time<= (a b)
  "check if given ros::time is less the or equal to the others"
  (not (ros::time> a b)))

(defclass ros::duration
  :super ros::time
  :slots ())
(defmethod ros::duration
  (:init (&optional sec nsec)
         (cond
          ((and (null sec) (null nsec))
           (setq sec 0 nsec 0))
          ((and sec (null nsec))
           (setq nsec (round (* (mod sec 1) 1000000000)))
           (setq sec  (floor sec))))
         (send-super :init :sec sec :nsec nsec))
  (:sleep () (ros::duration-sleep (send self :to-sec)))
  )

;; ROS 2 messages: the classes generated by gen_eus_ros2.py serialize the
;; CDR (XCDR1, little endian) payload, without the 4 bytes of the
;; encapsulation header, which is added / removed by roseus.so.
;; A class has (:serialization-length &optional (p 0)) -> end position,
;; (:serialize-cdr buf p) -> end position and (:deserialize-cdr buf p) ->
;; end position, positions are relative to the CDR alignment origin.
#+:ros2
(progn
(defun ros::cdr-align (p n)
  (let ((r (mod p n))) (if (= r 0) p (+ p (- n r)))))
(defun ros::cdr-put-string (buf p str wide)
  (setq p (ros::cdr-align p 4))
  (cond
   (wide ;; wstring: number of characters, 4 bytes per character
    (sys::poke (length str) buf p :integer) (incf p 4)
    (dotimes (i (length str)) (sys::poke (elt str i) buf p :integer) (incf p 4)))
   (t
    (sys::poke (1+ (length str)) buf p :integer) (incf p 4)
    (replace buf str :start1 p) (incf p (length str))
    (setf (elt buf p) 0) (incf p 1)))
  p)
(defun ros::cdr-get-string (buf p wide)
  "returns (string . end-position)"
  (let (n s)
    (setq p (ros::cdr-align p 4))
    (setq n (sys::peek buf p :integer)) (incf p 4)
    (cond
     (wide
      (setq s (make-string n))
      (dotimes (i n) (setf (elt s i) (logand (sys::peek buf p :integer) #xff)) (incf p 4)))
     (t
      (setq s (subseq buf p (+ p (max 0 (1- n)))))
      (incf p n)))
    (cons s p)))
(defun ros::cdr-int8 (v) (if (> v 127) (- v 256) v))
(defun ros::cdr-uint32 (v) (if (< v 0) (+ v #x100000000) v))
(defun ros::cdr-to-uint64 (v) (if (< v 0) (+ v (expt 2 64)) v))
(defun ros::cdr-uint64 (v) (if (>= v (expt 2 63)) (- v (expt 2 64)) v))
(defmethod ros::object
  (:serialize ()
   "returns serialized CDR payload of this message"
   (let ((buf (make-string (send self :serialization-length))))
     (send self :serialize-cdr buf 0)
     buf))
  (:deserialize (buf &optional (p 0))
   (send self :deserialize-cdr buf p)
   self))
;; builtin_interfaces/Time and Duration
(defmethod ros::time
  (:nanosec (&optional s) (send self :nsec s))
  (:serialization-length (&optional (p 0)) (+ (ros::cdr-align p 4) 8))
  (:serialize-cdr (buf p)
   (setq p (ros::cdr-align p 4))
   (sys::poke (send self :sec) buf p :integer)
   (sys::poke (send self :nsec) buf (+ p 4) :integer)
   (+ p 8))
  (:deserialize-cdr (buf p)
   (setq p (ros::cdr-align p 4))
   (send self :sec (sys::peek buf p :integer))
   (send self :nsec (ros::cdr-uint32 (sys::peek buf (+ p 4) :integer)))
   (+ p 8)))
)

;; anymsg
(defclass ros::anymsg
  :super ros::object
  :slots (_buff))

(defmethod ros::anymsg
  (:init ()
   (send-super :init)
   self)
;  (:serialization-length () )
;  (:serialize (&optional strm) )
  (:deserialize
   (buf &optional (ptr- 0))
   (setq _buff buf)
   self)
  (:buf ()
   _buff)
  )

(setf (get ros::anymsg :md5sum-) "*")
(setf (get ros::anymsg :datatype-) "*")
(setf (get ros::anymsg :definition-) "roseus/AnyMsg")

;;
(defclass timer-event
  :super propertied-object
  :slots (last-expected last-real current-expected current-real last-duration))
(defmethod timer-event
  (:init ()
    (setq last-expected (ros::time)
          last-real (ros::time)
          current-expected (ros::time)
          current-real (ros::time)
          last-duration (ros::time))
    self)
  (:prin1 (&optional (strm t) &rest msgs)
	    (send-super :prin1 strm
			(format nil "last-expected ~7,3f, last-real ~7,3f, current-expected ~7,3f, current-real ~7,3f, last-duration ~7,3f" (send last-expected :to-sec) (send last-real :to-sec) (send current-expected :to-sec) (send current-real :to-sec) (send last-duration :to-sec))))
  (:last-expected (&rest args) (user::forward-message-to last-expected args))
  (:last-real (&rest args) (user::forward-message-to last-real args))
  (:current-expected (&rest args) (user::forward-message-to current-expected args))
  (:current-real (&rest args) (user::forward-message-to current-real args))
  (:last-duration (&rest args) (user::forward-message-to last-duration args))
  )


;;


(defun ros::ros-home-dir ()
  (let ((ros-home (unix::getenv "ROS_HOME"))
        (ros-distro (unix::getenv "ROS_DISTRO")))
    (unless ros-home (setq ros-home (format nil "~A/.ros" (unix::getenv "HOME"))))
    (unless (probe-file (format nil "~A/roseus" ros-home))
      (unix::system (format nil "mkdir ~A/roseus" ros-home)))
    (unless (probe-file (format nil "~A/roseus/~A" ros-home ros-distro))
      (unix::system (format nil "mkdir ~A/roseus/~A" ros-home ros-distro)))
    (pushnew (format nil "~A/roseus/~A/" ros-home ros-distro) *load-path* :test #'equal)
    ros-home))
(defun ros::roseus-add-files (pkg type)
  (let ((dir (format nil "~A/~A/" (ros::find-load-msg-path pkg) type)))
    (unless (probe-file dir)
      (warning-message
       1 (format nil ";; can not find ~A directory for [~A] packages~%" dir pkg))
      (exit 1))
    (ros::ros-debug "Loading ~A" dir)
    (unless (find-package (string-upcase pkg))
      (make-package (string-upcase pkg)))
    (dolist (file (directory dir))
      (when (and (string= (pathname-type file) "l")
		 (not (= #\. (elt (pathname-name file) 0))))
	(let ((dirfile (concatenate string dir file))
              (modname (intern (string-upcase (format nil "~A/~A" pkg (pathname-name file))) *keyword-package*)))
	  (require modname dirfile)
          (when ros::*compile-message*
	    (in-package "ROS")
	    (let* ((fname (pathname-name dirfile))
		   (old-module (find (format nil "~A_~A" pkg fname)
				     *loaded-modules*
				     :key #'lisp::load-module-file-name
				     :test #'equal)))
              (unless old-module
                (comp::compile-file-if-src-newer dirfile (format nil "~A/~A_~A" dir pkg fname))
                (load (format nil "~A/~A_~A.so" dir pkg fname)
                      :entry (format nil "___~A" fname))
                ))
	    (in-package "USER"))
	    )))))

(defun ros::roseus-add-msgs (pkg) (ros::roseus-add-files pkg "msg"))
(defun ros::roseus-add-srvs (pkg) (ros::roseus-add-files pkg "srv"))

(unless (fboundp 'load-org-for-ros)
  (setf (symbol-function 'load-org-for-ros) (symbol-function 'load)))
(defun ros::resolve-ros-path (fname)
  (let* ((urlname (url-pathname fname))
	 (package-name (send urlname :host))
	 (path-name (namestring urlname))
	 (package-path (ros::rospack-find package-name)))
    (if (and package-path (= (elt package-path (1- (length package-path))) #\/)) ;; if end with /
        (setq package-path (subseq package-path 0 (1- (length package-path)))))
    (if (and path-name (= (elt path-name 0) #\/)) ;; if start with /
        (setq path-name (subseq path-name 1)))
    (if (and package-path path-name)
	(format nil "~A/~A" package-path path-name)
      (progn (warning-message 1 ";; could not find pacakge [~a]~%" package-name) nil))))

(defun load (fname &rest args)
  (let ((fullname fname))
    (when (and (stringp fname) (substringp "package://" fname))
      (setq fullname (ros::resolve-ros-path fname))
      (when ros::*compile-message*
        (let* ((urlname (url-pathname fname))
               (package-name (send urlname :host))
               (path-name (format nil "~A_~A" package-name (substitute #\_ #\/ (concatenate string (subseq (send urlname :directory-string) 1) (send urlname :name)))))
               (old-module (find path-name
                                 *loaded-modules*
                                 :key #'lisp::load-module-file-name
                                 :test #'equal)))
          (unless old-module
            (let* ((ppath (unix::getenv "CMAKE_PREFIX_PATH"))
                   (dir (format nil "~A/share/roseus/ros/~A" (subseq ppath 0 (position #\: ppath)) package-name)))
              (unless (probe-file dir) (unix:mkdir dir))
              (comp::compile-file-if-src-newer fullname (format nil "~A/~A" dir path-name))
              (return-from load (load (format nil "~A/~A.so" dir path-name)
                                      :entry (format nil "___~A" path-name)))))
          )))
    (if (null fullname) ;; when ros::resolve-ros-path is failed to load
        (error "file ~s not found" fname))
    (apply #'load-org-for-ros fullname args)))

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
     (setq ,sym (ros::get-param ,param-name ,default-value))
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

;;
;; load functions
;;
;; first look under "~A/roseus/${ROS_DISTRO}/${PKG}" and if not check CMAKE_PREFIX_PATH/share/roseus/ros/${PKG}
(defun ros::find-load-msg-path (pkg)
  (when (and (string= *eusdir* "/usr/share/euslisp/") (member pkg '("euslisp" "jskeus") :test #'string=))
    (warning-message 3 "on system installed euslisp, (ros::find-load-msg-path \"~A\") is skipped~%"
		     *eusdir* pkg)
    (return-from  ros::find-load-msg-path :no-msg-package))
  (let ((dirs (list ))
        (ppath (unix::getenv "CMAKE_PREFIX_PATH"))
        (s 0) i)
    (while (setq i (position #\: ppath :start s))
      (setq dirs (nconc dirs (list (format nil "~A/share" (subseq ppath s i)))))
      (setq s (1+ i)))
    (setq dirs (nconc dirs (list (format nil "~A/share" (subseq ppath s)))))
    ;;
    ;; INDIGO: https://github.com/jsk-ros-pkg/jsk_roseus/issues/554
    ;;          if pkg without msg, need to return no-msg-package
    ;; JADE/KINETIC: https://github.com/jsk-ros-pkg/jsk_robot/issues/823
    ;;          package without msg does not have manifest.l
    (dolist (dir dirs)
      (if (and (not (string= pkg "roseus"))
	       (probe-file (format nil "~A/~A/" dir pkg)) ;; package is in source tree
	       (not (probe-file (format nil "~A/roseus/ros/~A/" dir pkg)))) ;; but does not have manifest.l
	  (return-from ros::find-load-msg-path :no-msg-package))
      (if (probe-file (format nil "~A/roseus/ros/~A/" dir pkg))
	  (return-from ros::find-load-msg-path (format nil "~A/roseus/ros/~A" dir pkg))))
    (ros::ros-error "Could not find roseus messages for ~A under ~A~%try rosrun roseus generate-all-msg-srv.sh ~A" pkg dirs pkg)
    nil))

(defun ros::load-ros-manifest (pkg)
  (let* ((msg-path (ros::find-load-msg-path pkg))
	 (manifest (format nil "~A/manifest.l" msg-path)))
    (cond ((eq msg-path :no-msg-package)
           ;; on jade/kinetic, package without msg does not have manifest.l
	   (ros::ros-warn "Calling (load-ros-manifest ~A) for the package without msg/srv will be deprecated" pkg)
	   (ros::ros-warn "ACTION REQUIRED")
	   (ros::ros-warn " Use (ros::load-ros-package ~A) or (ros::roseus-add-msgs/srvs ~A) for the dependent packages with msg/srv" pkg pkg)
	   (ros::ros-warn " See https://github.com/jsk-ros-pkg/jsk_robot/issues/823")
           (dolist (dep (ros::rospack-depends pkg))
             (ros::load-ros-package dep))
           )
	  ((probe-file manifest)
	   (ros::ros-debug "Loading ~A" manifest)
	   (load-org-for-ros manifest))
	  (t
	   (ros::ros-error "Could not find ~A~%try rosrun roseus generate-all-msg-srv.sh ~A" manifest pkg)))))

(defun ros::load-ros-package (pkg)
  "load required roseus files for given package"
  (unless (find-package (string-upcase pkg))
    (make-package (string-upcase pkg)))
  (when (probe-file (format nil "~A/msg" (ros::find-load-msg-path pkg)))
    (ros::roseus-add-msgs pkg))
  (when (probe-file (format nil "~A/srv" (ros::find-load-msg-path pkg)))
    (ros::roseus-add-srvs pkg)))

;; ROS 2: euslisp messages are in <prefix>/share/roseus/ros/<pkg> (generated
;; when building the package), or in $ROS_HOME/roseus/$ROS_DISTRO/<pkg>,
;; which are generated by gen_eus_ros2.py at the first time they are
;; loaded, and regenerated when the interfaces are updated.
#+:ros2
(progn
(defun ros::ament-prefix-path ()
  (let ((ppath (unix::getenv "AMENT_PREFIX_PATH")) (s 0) i dirs)
    (when ppath
      (while (setq i (position #\: ppath :start s))
        (push (subseq ppath s i) dirs)
        (setq s (1+ i)))
      (push (subseq ppath s) dirs))
    (remove "" (nreverse dirs) :test #'string=)))
(defun ros::interface-files (share)
  (let (files)
    (dolist (sub '("msg" "srv" "action"))
      (let ((dir (format nil "~A/~A" share sub)))
        (when (probe-file dir)
          (dolist (f (directory dir))
            (when (member (pathname-type f) '("msg" "srv" "action") :test #'equal)
              (push (format nil "~A/~A" dir f) files))))))
    files))
(defun ros::generate-msg-package (pkg outdir)
  (let ((script (format nil "~A/../../lib/rosidl_generator_eus/gen_eus_ros2.py"
                       (ros::rospack-find "rosidl_generator_eus"))))
    (ros::ros-info "generating euslisp messages of ~A in ~A" pkg outdir)
    (unix::system (format nil "python3 ~A -o ~A ~A" script outdir pkg))))
(defun ros::find-load-msg-path (pkg)
  (when (and (string= *eusdir* "/usr/share/euslisp/") (member pkg '("euslisp" "jskeus") :test #'string=))
    (return-from ros::find-load-msg-path :no-msg-package))
  (let* ((share (ros::rospack-find pkg))
         (files (if share (ros::interface-files share)))
         (newest (if files (apply #'max (mapcar #'file-write-date files)) 0))
         (home (format nil "~A/roseus/~A" (ros::ros-home-dir) (unix::getenv "ROS_DISTRO")))
         (dir (format nil "~A/~A" home pkg)))
    ;; generated when the package was built
    (dolist (prefix (ros::ament-prefix-path))
      (let ((d (format nil "~A/share/roseus/ros/~A" prefix pkg)))
        (when (and (probe-file d) (>= (file-write-date d) newest))
          (return-from ros::find-load-msg-path d))))
    (unless files
      (return-from ros::find-load-msg-path
        (if (probe-file dir) dir :no-msg-package)))
    ;; generated in $ROS_HOME
    (when (or (not (probe-file dir)) (< (file-write-date dir) newest))
      (ros::generate-msg-package pkg home))
    (if (probe-file dir)
        dir
      (progn
        (ros::ros-error "Could not generate roseus messages for ~A in ~A" pkg home)
        nil))))
(defun ros::load-ros-package (pkg)
  "load required roseus files for given package"
  (unless (find-package (string-upcase pkg))
    (make-package (string-upcase pkg)))
  (let ((path (ros::find-load-msg-path pkg)))
    (when (stringp path)
      (dolist (type '("msg" "srv" "action"))
        (when (probe-file (format nil "~A/~A" path type))
          (ros::roseus-add-files pkg type))))))
(defun ros::roseus-add-actions (pkg) (ros::roseus-add-files pkg "action"))
(defun ros::load-ros-manifest (pkg)
  (let ((path (ros::find-load-msg-path pkg)))
    (cond ((stringp path)
           (ros::load-ros-package pkg))
          (t
           (dolist (dep (ros::rospack-depends pkg))
             (when (stringp (ros::find-load-msg-path dep))
               (ros::load-ros-package dep)))))))
)

(setq sys::*exit-hook* 'ros::exit)

(provide :roseus "roseus.l")

(in-package "USER")
