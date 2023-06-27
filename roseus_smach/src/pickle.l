;;
;; simple pickler for python debugging
;; this script only dump
;;
;; pickle protocol version 0.
;; this pickle class does not memorize object.(originally fast option)
;;

(make-package "PICKLE")
(in-package "PICKLE")

;; To see detail, please read pickle.py
(defconstant PICKLE::_MARK         #\()
(defconstant PICKLE::_STOP         #\.)

(defconstant PICKLE::_FLOAT        #\F)
(defconstant PICKLE::_INT          #\I)
(defconstant PICKLE::_STRING       #\S)
(defconstant PICKLE::_LIST         #\l)
(defconstant PICKLE::_APPEND       #\a)
(defconstant PICKLE::_TUPLE        #\t)
(defconstant PICKLE::_DICT         #\d)
(defconstant PICKLE::_SETITEM      #\s)
(defconstant PICKLE::_T           #x88)
(defconstant PICKLE::_NIL         #x89)

(defclass pickler
  :super propertied-object
  :slots (ss dict))
(defmethod pickler
  (:init (&key ((:dict dct) nil)) (setq dict dct))
  (:dump
   (obj &optional (strm nil))
   (if strm (setq ss strm)
     (setq ss (make-string-output-stream 4096)))
   (send self :save obj)
   (format ss "~c" _STOP)
   (if strm (setq ss nil)
     (get-output-stream-string ss)))
  (:save (obj)
   (cond ((listp obj)
    (if (and (cdr obj) (atom (cdr obj)))
        (send self :save-cons obj)
;;    (if (every #'(lambda(it)(and (consp it) (cdr it) (atom (cdr it)))) obj)
      (if (and dict (every #'(lambda (x) (and (consp x) (not (consp (car x))))) obj))
          (send self :save-dict obj) ;; all items are cons
        (send self :save-list obj))))
         ((stringp obj) (send self :save-string obj))
         ((vectorp obj) (send self :save-vector obj))
         ((floatp obj) (send self :save-float obj))
         ((integerp obj) (send self :save-int obj))
         ((or (eq t obj) (eq nil obj)) (send self :save-bool obj))
         ((symbolp obj) (send self :save-string (string obj)))
         (t (send self :save-string (format nil "~a" obj)))
         ))
  (:save-bool (obj) (format ss "~c" (if obj _T _NIL)))
  (:save-int (obj) (format ss "~c~a~c" _INT obj #\newline))
  (:save-float (obj) (format ss "~c~a~c" _FLOAT obj #\newline))
  (:save-string (obj) (format ss "~c~c~a~c~c" _STRING #\' obj #\' #\newline))
  (:save-list
   (obj)
   (format ss "~c~c" _MARK _LIST)
   (dolist (item obj)
     (send self :save item)
     (format ss "~c" _APPEND)))
  (:save-vector ;; save vector as a tuple
   (obj)
   (when (eq 0 (length obj))
     (format ss "~c~c" _MARK _TUPLE)
     (return-from :save-vector nil))
   (format ss "~c" _MARK)
   (dotimes (i (length obj))
     (send self :save (elt obj i)))
   (format ss "~c" _TUPLE))
  (:save-cons ;; save cons as a dictionary
   (obj)
   (format ss "~c~c" _MARK _DICT)
   (send self :save (car obj))
   (send self :save (cdr obj))
   (format ss "~c" _SETITEM))
  (:save-dict ;; save list of cons as a dictionary
   (obj)
   (format ss "~c~c" _MARK _DICT)
   (dolist (item obj)
     (send self :save (car item))
     (send self :save (cdr item))
     (format ss "~c" _SETITEM)))
  )


(defclass unpickler
  :super propertied-object
  :slots (eof stack mark))
(defmethod unpickler
  (:init
   (&key ((:dict dct) nil))
   (setq eof (gentemp "EOF"))
   (setq mark (gentemp "MARK"))
   (setq stack nil))
  (:read
   (obj &optional (strm nil))
   (let (ret)
     (labels
      ((dictp (x) (and (consp x) (= (length x) 1) (cdr x) (atom (cdr x)) (car x)))
       (deep-reverse (x) (map cons #'(lambda (x) (cond ((dictp x) (cons (cdr x) (if (consp (car x)) (deep-reverse (car x)) (car x))))
						       ((consp x) (deep-reverse x))
						       (t x)))
			      (reverse x)))
       )
#+:debug
      (progn
	(format t "reading [")
	(dotimes (i (length obj))
	  (format t "~d(~c) " (elt obj i) (elt obj i)))
	(format t "]~%"))
      (setq ret (send self :load (make-string-input-stream obj)))
      (if (consp ret)
	  (deep-reverse ret)
	ret)
      )))
  (:load
   (is)
   (let (c)
     (catch :STOP
       (while (not (eq (setq c (read-char is nil eof)) eof))
	 ;; (format t "LOAD stack:~A (~c)~%" stack c)
	 (case c
	       ((#._INT #\L)
		(send self :load-int is))
	       (#._FLOAT
		(send self :load-float is))
	       (#._STRING
		(send self :load-string is))
	       (#\J   ;; BININT  push four-byte signed int
		(send self :load-binint is))
	       (#\K   ;; BININT1 push 1-byte unsigned int
		(send self :load-binuint is 1))
	       (#\M   ;; BININT2 push 2-byte unsigned int
		(send self :load-binuint is 2))
	       (#x8a  ;; LONG1 push long from < 256 bytes
		(send self :load-binuint is (read-char is nil eof)))
	       (#\G   ;; BINFLOAT push float; arg is 8-byte float encoding
		(send self :load-binfloat is))
	       ((#\T #\U)   ;; BINSTRING, SHORT_BINSTRING  push string; counted binary string argument
		(send self :load-binstring is))
	       (#\N   ;; NONE push None
		(send self :load-none is))
	       (#\p   ;; PUT   store stack top in memo; index is string arg
		(while (/= (setq c (read-char is nil eof)) #\newline)))
	       (#\q   ;; BINPUT   store stack top in memo; index is 1-byte arg
		(read-char is nil eof))
	       (#x88  ;; NEWTRUE (136)  push True
		(push t stack))
	       (#x89  ;; NEWFALSE (137)  push False
		(push nil stack))
	       (#._MARK
		(send self :load-mark is))
	       ((#._LIST #\t)  ;; TUPLE  build tuple from topmost stack items
		(send self :load-list is))
	       ((#\] #\})   ;; EMPTY_LIST push empty list, EMPTY_DICT push empty dict
		(push nil stack))
	       (#._DICT
		(send self :load-dict is))
	       (#._SETITEM
		(send self :load-setitem is))
	       (#\u
		(send self :load-setitems is))
	       (#._APPEND
		(send self :load-append is))
	       (#\e   ;; APPENDS  extend list on stack by topmost stack slice
		(send self :load-appends is))
	       (#._STOP
		(send self :load-stop is))
	       (#x80  ;; PROTO  identify pickle protocol
		(read-char is nil eof))  ;; protocol version
	       (#x85  ;; TUPLE1  build 1-tuple from three topmost stack items
		(send self :load-append-n is 1))
	       (#x86  ;; TUPLE2  build 2-tuple from three topmost stack items
		(send self :load-append-n is 2))
	       (#x87  ;; TUPLE3  build 3-tuple from three topmost stack items
		(send self :load-append-n is 3))
	       (t
		(warning-message 1 "Unknown string ~c(~d)~%" c c)))))))
  (:load-int
   (is)
   (let (c str (ss (make-string-output-stream 4096)))
     (while (/= (setq c (read-char is nil eof)) #\newline)
       (format ss "~c" c))
     (setq str (get-output-stream-string ss))
     (push
      (cond ((string= str "00") nil)
	    ((string= str "01") t)
	    (t (read-from-string (string-right-trim "L" str))))
      stack)))
  (:load-float
   (is)
   (let (c (ss (make-string-output-stream 4096)))
     (while (/= (setq c (read-char is nil eof)) #\newline)
       (format ss "~c" c))
     (push (read-from-string (get-output-stream-string ss)) stack)))
  (:load-string
   (is)
   (let (c (ss (make-string-output-stream 4096)))
     (while (/= (setq c (read-char is nil nil)) #\newline)
       (case c
	     (#\'
	      (format ss "\""))
	     (#\\
	      (if (setq c (read-char is nil nil))
		  (case c
			(#\n
			 (format ss "~c" #\newline))
			(#\t
			 (format ss "~c" #\tab))
			(t
			 (warning-message 3 "Unknown string tag \~c(~d)~%" c c)))))
	     (t
	      (format ss "~c" c))))
     (push (read-from-string (get-output-stream-string ss)) stack)))
  (:load-binint
   (is)
   (let (c (r 0))
     (dotimes (i 4)
       (setq c (read-char is nil eof))
       (incf r (* c (expt 256 i))))
     (push (if (< r #x80000000) r (- r #x100000000)) stack)))
  (:load-binuint
   (is n)
   (let (c (r 0))
     (dotimes (i n)
       (setq c (read-char is nil eof))
       (incf r (* c (expt 256 i))))
     (push r stack)))
  (:load-binfloat
   (is)
   (let ((buf "01234567"))
     (dotimes (i 8)
       (setf (elt buf (- 7 i)) (read-char is nil eof)))
     (push (sys::peek buf 0 :double) stack)))
  (:load-binstring
   (is)
   (let (c n (ss (make-string-output-stream 4096)))
     (setq n (read-char is nil nil))
     (dotimes (i n)
       (setq c (read-char is nil nil))
       (case c
	     (#\'
	      (format ss "\""))
	     (#\\
	      (if (setq c (read-char is nil nil))
		  (case c
			(#\n
			 (format ss "~c" #\newline))
			(#\t
			 (format ss "~c" #\tab))
			(t
			 (warning-message 3 "Unknown string tag \~c(~d)~%" c c)))))
	     (t
	      (format ss "~c" c))))
     (push (get-output-stream-string ss) stack)))
  (:load-none
   (is)
   nil)
  (:load-mark
   (is)
   (push mark stack))
  (:marker
   ()
   (let ((k 0))
     (while (not (eq (elt stack k) mark))
       (incf k))
     k))
  (:load-list
   (is)
   (let ((k (send self :marker)))
     (setq stack (concatenate cons (list (subseq stack 0 k)) (if (< (1+ k) (length stack)) (subseq stack (1+ k)) (list ))))
     ))
  (:load-dict
   (is)
   (let* ((k (send self :marker))
	  (items (subseq stack 0 k))
	  (dict nil))
     (do ((j 0)) ((>= j (length items)))
	 (push (cons (elt items j) (elt items (1+ j))) dict)
	 (incf j 2))
     (setq stack (concatenate cons (list dict) (if (< (1+ k) (length stack)) (subseq stack (1+ k)) (list ))))
     ))
  (:load-setitem
   (is)
   (let* ((key (pop stack))
	  (value (pop stack)))
     (push (cons key value) (car stack))))
  (:load-setitems
   (is)
   (let (items key value)
     (while (not (eq (car stack) mark))
       (setq key (pop stack))
       (setq value (pop stack))
       (push (cons key value) items))
     (nreverse items)
     (pop stack) ;; remove marker
     (setf (car stack) (append (car stack) items))
     ))
  (:load-append
   (is)
   (let ((item (pop stack)))
     (push item (car stack))
     ))
  (:load-appends
   (is)
   (let (items)
     (while (not (eq (car stack) mark))
       (push (pop stack) items))
     (nreverse items)
     (pop stack) ;; remove marker
     (setf (car stack) (append (car stack) items))
     ))
  (:load-append-n
   (is len)
   (let (items)
     (print stack)
     (dotimes (i len)
       (push (pop stack) items))
     (nreverse items)
     (push items stack)
     ))
  (:load-stop
   (is)
   (throw :STOP (pop stack)))
  ;;
  )

(shadow 'dump (find-package "PICKLE"))
(shadow 'load (find-package "PICKLE"))

(defun pickle::dump (obj &optional (strm nil))
  (send (instance pickler :init :dict t) :dump obj strm ))

(defun pickle::load (obj)
  (send (instance unpickler :init :dict t) :read obj))

(provide :pickle "pickle.l")
