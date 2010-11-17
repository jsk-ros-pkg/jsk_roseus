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
;		  (if (every #'(lambda(it)(and (consp it) (cdr it) (atom (cdr it)))) obj)
		  (if (and dict (every #'consp obj))
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

(defun pickle::dump (obj &optional (strm nil))
  (send (instance pickler :init :dict t) :dump obj strm ))

(provide :pickle "pickle.l")
