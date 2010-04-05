(defun generate-msg-type-name-list (strm)
  (let (msgs consts line pos type var init-elem vec-size)
    (while (not (eq :eof (setq line (read-line strm nil :eof))))
      (setq line (nstring-downcase line))
      ;; remove comments
      (if (string= line "---") (return))
      (setq pos (position #\# line))
      (if pos (setq line (subseq line 0 pos)))
      ;; replcae tab to whitespace
      (setq line (nsubstitute #\space #\tab line))
      ;; remove whitespaces
      (setq line (string-trim " " line))

      ;;(setq pos (count #\space line))
      ;;(setq line (remove-duplicates  line :test #'(lambda (a b) (if (eq a #\space) (equal a b) nil))))
      ;;
      (when (> (length line) 0)
	;; split line into type and var
	(setq pos (position #\space line))
	(setq type (subseq line 0 pos))
	(setq var (subseq line (1+ pos)))
	;; format type
	(if (string= "header" type)
	    (setq type (concatenate string "roslib::" type)))
	(if (setq pos (position #\/ type))
	    (setq type (format nil "~A::~A" (subseq type 0 pos) (subseq type (1+ pos)))))
	;; when variable is list
	(if (setq pos (position #\[ type))
	    (setq vec-size
		  (if (= (1+ pos) (position #\] type)) 0
		    (read-from-string (subseq type (1+ pos))))
		  type (subseq type 0 pos))
	  (setq vec-size nil))
	;; constants
	(if (setq pos (position #\= var))
	    (progn
              (setq init-elem (string-trim " " (subseq var (1+ pos))))
              (setq var (string-trim " " (subseq var 0 pos)))
              (push (list type var init-elem vec-size) consts))
	  (push (list type var vec-size) msgs))))
    (list (setq msgs (nreverse msgs)) (setq consts (nreverse consts)))))

(defun generate-msg-output-file-name (fname pkg name dir)
  (let (l oname)
    (setq l (read-line (piped-fork (format nil "rospack find ~A" pkg)) nil))
    (unless l
      (warning-message 1 ";; ERROR could not find ~A package~%" pkg)(exit 1))
    (setq oname (format nil "~A/~A/eus/~A/" l dir pkg))
    (unless (lisp::directory-p oname)
      (warning-message 6 ";; mkdir ~A~%" oname)
      (unix::system (format nil "mkdir -p ~A" oname)))
    (setq oname (concatenate string oname (format nil "~A-~A.l" pkg name)))))

;;
;;
;;

(defun generate-variable-type (type)
  (cond ((or (string= type "byte") (string= type "char")
	     (string= type "int8") (string= type "uint8"))
	 :int8)
	((or (string= type "int16") (string= type "uint16"))
	 :int16)
	((or (string= type "int32") (string= type "uint32"))
	 :int32)
	((or (string= type "int64") (string= type "uint64"))
	 :int64)
	((string= type "float32")
	 :float32)
	((string= type "float64")
	 :float64)
	((string= type "string")
	 :string)
	((or (string= type "time") (string= type "duration"))
	 :time)
	(t
	 :rostype)))

;;
;;
;;

(defun generate-msg-headers (strm pkg name)
  (let ()
    (format strm ";;~%")
    (format strm ";; Auto-generated from ~A/~A.msg. ~%" pkg name)
    (format strm ";; Do not edit.~%")
    (format strm ";;~%")
    (format strm "~%")
    (format strm "(if (boundp '~A::~A)~%" pkg name)
    (format strm "  (shadow '~A (find-package \"~A\")))~%" name (string-upcase pkg))
    (format strm "~%")
    ))

(defun generate-msg-defconstants (strm pkg name consts)
  (let ()
    (dolist (const consts)
      (format strm "(defconstant ~A::~A ~A)~%" pkg (cadr const) (caddr const)))
    (format strm "~%")
    ))

(defun generate-msg-defclass (strm pkg name msgs)
  (let ()
    (format strm "(defclass ~A::~A~%" pkg name)
    (format strm "  :super ros::object~%")
    (format strm "  :slots (")
    (dolist (msg msgs) (format strm "~A " (cadr msg)))
    (format strm "))~%")
    (format strm "~%")
    ))

(defun generate-msg-init (strm pkg name msgs)
  (let (type var var-type init-elem vec-size)
    (format strm "  (:init~%")
    (format strm "   (&key~%")
    (dolist (msg msgs)
      (setq type (elt msg 0) var (elt msg 1)
	    vec-size (elt msg 2))
      (setq var-type (generate-variable-type type))
      (setq init-elem
	    (case var-type
		  ((:int8 :int16 :int32 :int64) 0)
		  ((:float32 :float64) 0.0)
		  (:string "\"\"")
		  (:time "(instance ros::rostime :init)")
		  (:rostype (format nil "(instance ~A :init)"
				    (if (substringp "::" type)
					type
				      (concatenate string pkg "::" type))))
		  (t (error "undefined type ~A~%" msg))))
      (cond
       ((and vec-size (memq var-type '(:int8 :int16 :int32 :int64)))
	(format strm "    ((:~A _~A) (make-array ~A :initial-element ~A :element-type :integer))~%" var var vec-size init-elem))
       ((and vec-size (memq var-type '(:float32 :float64)))
	(format strm "    ((:~A _~A) (make-array ~A :initial-element ~A :element-type :float))~%" var var vec-size init-elem))
       (vec-size
	(format strm "    ((:~A _~A) (let (r) (dotimes (i ~A) (push ~A r)) r))~%" var var vec-size init-elem))
       (t
	(format strm "    ((:~A _~A) ~A)~%" var var init-elem))))
    (format strm "    )~%")
    (format strm "   (send-super :init)~%")
    (dolist (msg msgs)
      (setq type (elt msg 0) var (elt msg 1)
	    vec-size (elt msg 2))
      (let ((s1 "") (s2 ""))
	(unless vec-size
	  (case
	   (generate-variable-type type)
	   ((:int8 :int16 :int32 :int64) (setq s1 "(round " s2 ")"))
	   ((:float32 :float64) (setq s1 "(float " s2 ")"))
	   (:string (setq s1 "(string " s2 ")"))))
	(format strm "   (setq ~A ~A_~A~A)~%" var s1 var s2)))
    (format strm "   self)~%")))

(defun generate-msg-accessors (strm pkg name msgs)
  (let (var)
    (dolist (msg msgs)
      (setq var (cadr msg))
      (format strm "  (:~A~%" var)
      (if (eq (generate-variable-type (car msg)) :rostype)
	  (progn
	    (format strm "   (&rest _~A)~%" var)
	    (format strm "   (if (keywordp (car _~A))~%" var)
	    (format strm "       (send* ~A _~A)~%" var var)
	    (format strm "     (progn~%")
	    (format strm "       (if _~A (setq ~A (car _~A)))~%" var var var)
	    (format strm "       ~A)))~%" var))
	(progn
	  (format strm "   (&optional _~A)~%" var)
	  (format strm "   (if _~A (setq ~A _~A)) ~A)~%" var var var var))))
    ))

(defun generate-msg-serialization-length (strm pkg name msgs)
  (let (type var vec-size gen-type)
    (format strm "  (:serialization-length~%")
    (format strm "   ()~%")
    (format strm "   (+~%")
    (dolist (msg msgs)
      (setq type (elt msg 0) var (elt msg 1)
	    vec-size (elt msg 2)
            gen-type (generate-variable-type type))
      (cond
       (vec-size (format strm "    ;; ~A[~A] ~A~%" type (if (zerop vec-size) "" vec-size) var))
       (t        (format strm "    ;; ~A ~A~%" type var)))
      (format strm "    ")
      (if (and vec-size (not (eq gen-type :rostype))) (format strm "(* "))
      (case
       gen-type
       (:int8 (format strm "1"))
       (:int16 (format strm "2"))
       ((:int32 :float32) (format strm "4"))
       ((:int64 :float64) (format strm "8"))
       (:string (format strm "4 (length ~A)"  var))
       (:time (format strm "8"))
       (:rostype
        (if vec-size
            (format strm "(apply #'+ (send-all ~A :serialization-length))" var)
          (format strm "(send ~A :serialization-length)" var)))
       (t (error "undefined type ~A~%" msg)))
      (cond
       ((and vec-size (eq gen-type :rostype) (zerop vec-size))
        (format strm " 4"))
       ((and vec-size (not (eq gen-type :rostype)) (zerop vec-size))
        (format strm " (length ~A)) 4" var))
       ((and vec-size (not (eq gen-type :rostype)))
        (format strm " ~A)" vec-size)))
      (format strm "~%")
      )
    (format strm "    ))~%")
    ))

(defun generate-msg-serialize (strm pkg name msgs)
  (let (type var vec-size)
    (format strm "  (:serialize~%")
    (format strm "   (&optional strm)~%")
    (format strm "   (let ((s (if strm strm~%")
    (format strm "              (make-string-output-stream (send self :serialization-length)))))~%")
    (dolist (msg msgs)
      (setq type (elt msg 0) var (elt msg 1)
	    vec-size (elt msg 2))
      (cond
       ((and vec-size (zerop vec-size))
	(format strm "     ;; ~A[] ~A~%" type var)
	(format strm "     (write-long (length ~A) s)~%" var)
	(format strm "     (dotimes (i (length ~A))~%  " var)
	(setq var (format nil "(elt ~A i)" var)))
       (vec-size
	(format strm "     ;; ~A[~A] ~A~%" type vec-size var)
	(format strm "     (dotimes (i ~A)~%  " vec-size)
	(setq var (format nil "(elt ~A i)" var)))
       (t
        (format strm "     ;; ~A ~A~%" type var)))
      (case
       (generate-variable-type type)
       (:int8
	(format strm "     (write-byte ~A s)~%" var))
       (:int16
	(format strm "     (write-word ~A s)~%" var))
       (:int32
	(format strm "     (write-long ~A s)~%" var))
       (:int64
	(format strm "     (sys::poke ~A (send s :buffer) (send s :count) :long) (incf (stream-count s) 8)~%" var))
       (:float32
	(format strm "     (sys::poke ~A (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)~%" var))
       (:float64
	(format strm "     (sys::poke ~A (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)~%" var))
       (:string
	(format strm "     (write-long (length ~A) s) (princ ~A s)~%" var var))
       (:time
	(format strm "     (write-long (send ~A :sec) s) (write-long (send ~A :nsec) s)~%" var var))
       (:rostype
	(format strm "     (send ~A :serialize s)~%" var))
       (t (error "undefined type ~A~%" msg)))
      (if vec-size (format strm "       )~%"))
      )
    (format strm "     ;;~%")
    (format strm "     (if (null strm) (get-output-stream-string s))))~%")))

(defun generate-msg-deserialize (strm pkg name msgs)
  (let (type var vec-size set)
    (format strm "  (:deserialize~%")
    (format strm "   (buf)~%")
    (format strm "   (let ((cnt 0))~%")
    (dolist (msg msgs)
      (setq type (elt msg 0) var (elt msg 1)
	    vec-size (elt msg 2))
      (cond
       ((and vec-size (zerop vec-size))
	(format strm "     ;; ~A[] ~A~%" type var)
	(format strm "     (let (n)~%")
	(format strm "       (setq n (sys::peek buf cnt :integer)) (incf cnt 4)~%")
	(case
	 (generate-variable-type type)
	 ((:int8 :int16 :int32 :int64)
	  (format strm "       (setq ~A (instantiate integer-vector n))~%" var))
	 ((:float32 :float64)
	  (format strm "       (setq ~A (instantiate float-vector n))~%" var))
	 (:rostype
	  (format strm "       (setq ~A (let (r) (dotimes (i n) (push (instance ~A :init) r)) r))~%" var (if (substringp "::" type) type (concatenate string pkg "::" type)))))
	(format strm "       (dotimes (i n)~%" var)
	(format strm "         ")
	(setq set "setf" var (format nil "(elt ~A i)" var)))
       (vec-size
	(format strm "     ;; ~A[~A] ~A~%" type vec-size var)
	(format strm "     (dotimes (i (length ~A))~%" var)
	(format strm "       ")
	(setq set "setf" var (format nil "(elt ~A i)" var)))
       (t
        (format strm "     ;; ~A ~A~%" type var)
        (format strm "     ")
	(setq set "setq")))
 
      (case
       (generate-variable-type type)
       (:int8
	(format strm "(~A ~A (sys::peek buf cnt :char)) (incf cnt 1)~%" set var))
       (:int16
	(format strm "(~A ~A (sys::peek buf cnt :short)) (incf cnt 2)~%" set var))
       (:int32
	(format strm "(~A ~A (sys::peek buf cnt :integer)) (incf cnt 4)~%" set var))
       (:int64
	(format strm "(~A ~A (sys::peek buf cnt :long)) (incf cnt 8)~%" set var))
       (:float32
	(format strm "(~A ~A (sys::peek buf cnt :float)) (incf cnt 4)~%" set var))
       (:float64
	(format strm "(~A ~A (sys::peek buf cnt :double)) (incf cnt 8)~%" set var))
       (:string
	(format strm "(let (n) (setq n (sys::peek buf cnt :integer)) (incf cnt 4) (~A ~A (subseq buf cnt (+ cnt n))) (incf cnt n))~%" set var))
       (:time
	(format strm "(send ~A :sec (sys::peek buf cnt :integer)) (incf cnt 4)  (send ~A :nsec (sys::peek buf cnt :integer)) (incf cnt 4)~%" var var))
       (:rostype
	(format strm "(~A ~A (send ~A :deserialize (subseq buf cnt))) (incf cnt (send ~A :serialization-length))~%" set var var var))
       (t (error "undefined type ~A~%" msg)))
      (cond
       ((and vec-size (zerop vec-size))
	(format strm "         ))~%"))
       (vec-size
	(format strm "       )~%")))
      )
    (format strm "     ;;~%")
    (format strm "     self))~%")))

(defun generate-msg-md5sum (strm pkg name fname)
  (let ()
    (format strm "(setf (get ~A::~A :md5sum) \"~A\")~%" pkg name
	    (read-line (piped-fork (format nil "`rospack find roslib`/scripts/gendeps --md5 ~A" fname))))))

(defun generate-msg-type (strm pkg name)
  (let ()
    (format strm "(setf (get ~A::~A :type) \"~A/~A\")~%" pkg name pkg name)
    (format strm "~%")))

(defun generate-srv-type (strm pkg name req-msgs res-msgs)
  (let ()
    (format strm "(setf (get ~A::~A :request) ~A::~A)~%" pkg name pkg req-msgs)
    (format strm "(setf (get ~A::~A :response) ~A::~A)~%" pkg name pkg res-msgs)
    (format strm "~%")
    (format strm "(defmethod ~A::~A~%" pkg req-msgs)
    (format strm "  (:response () (instance ~A::~A :init)))~%" pkg res-msgs)
    (format strm "~%")))


;;;
;;;
;;;

(defun genfile-header (fname)
  (let (pkg name)
    (setq pkg (car (last (butlast (pathname-directory fname)))))
    (setq name (pathname-name fname))
    (warning-message 6 ";; generate message for ~A/~A~%" pkg name)
    (warning-message 6 ";; parsing ~A~%" fname)
    (unless (probe-file fname)
      (warning-message 1 ";; ERROR message file ~A not found~%" fname)(exit 1))
    (list pkg name)))

(defun genfile-body (pkg name msgs strm)
  (let ()
    ;;
    ;; defclass
    (generate-msg-defclass strm pkg name msgs)
    ;; defmethod
    (format strm "(defmethod ~A::~A~%" pkg name)
    ;; :init
    (generate-msg-init strm pkg name msgs)
    ;; :accessors
    (generate-msg-accessors strm pkg name msgs)
    ;; serialization-length
    (generate-msg-serialization-length strm pkg name msgs)
    ;; serialize
    (generate-msg-serialize strm pkg name msgs)
    ;; deserialize
    (generate-msg-deserialize strm pkg name msgs)
    (format strm "  )~%~%")
    ;;
    ))

;;;
;;;
;;;

(defun genmsg-eus (fname)
  (let (msgs consts pkg name oname)
    (multiple-value-setq (pkg name) (genfile-header fname))
    (with-open-file
     (strm fname :direction :input)
     (multiple-value-setq (msgs consts) (generate-msg-type-name-list strm)))

    (setq oname (generate-msg-output-file-name fname pkg name "msg"))
    (warning-message 6 ";; writing to ~A~%" oname)

    (with-open-file
     (strm oname :direction :output)
     (generate-msg-headers strm pkg name)
     ;; defconstants
     (generate-msg-defconstants strm pkg name consts)
     ;;
     (genfile-body pkg name msgs strm)
     ;; md5sum/type
     (generate-msg-md5sum strm pkg name fname)
     (generate-msg-type strm pkg name)
     (format strm "~%~%")
     )))

(defun genfile-srvs (pkg name strm)
  (let ()
    (format strm "(defclass ~A::~A~%" pkg name)
    (format strm "  :super ros::object~%")
    (format strm "  :slots ())~%~%")
#|
    (format strm "(defmethod ~A::~A~%" pkg name)
    (format strm "  (:init ()~%")
    (format strm "   (setq request (instance ~A::~ARequest :init)~%" pkg name)
    (format strm "         response (instance ~A::~AResponse :init)))~%" pkg name)
    (format strm "  (:request  (&rest args) (if args (send* request args) request))~%")
    (format strm "  (:response (&rest args) (if args (send* response args) response))~%")
    (format strm ")~%~%")
|#
    ))

(defun gensrv-eus (fname)
  (let (req-msgs res-msgs consts pkg name req-name res-name oname)
    (multiple-value-setq (pkg name) (genfile-header fname))
    (setq req-name (format nil "~ARequest"  name)
          res-name (format nil "~AResponse" name))
    (with-open-file
     (strm fname :direction :input)
     (multiple-value-setq (req-msgs consts) (generate-msg-type-name-list strm))
     (setq res-msgs (car (generate-msg-type-name-list strm))))

    (setq oname (generate-msg-output-file-name fname pkg name "srv"))
    (warning-message 6 ";; writing to ~A~%" oname)

    (with-open-file
     (strm oname :direction :output)
     (generate-msg-headers strm pkg name)
     ;; defconstants
     (generate-msg-defconstants strm pkg name consts)
     ;;
     (genfile-body pkg req-name req-msgs strm)
     (genfile-body pkg res-name res-msgs strm)
     ;; md5sum/type
     (genfile-srvs pkg name strm)
     (generate-msg-md5sum strm pkg name fname)
     (generate-msg-type strm pkg name)
     (generate-srv-type strm pkg name req-name res-name)
     (generate-msg-md5sum strm pkg req-name fname)
     (generate-msg-type   strm pkg req-name)
     (generate-msg-md5sum strm pkg res-name fname)
     (generate-msg-type   strm pkg res-name)
     (format strm "~%~%")
     )))


