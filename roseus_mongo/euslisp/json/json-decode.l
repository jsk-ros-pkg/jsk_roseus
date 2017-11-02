;; json-decode.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(unless (find-package "JSON") (make-package "JSON"))
(in-package "JSON")

(defun string->keyword (str)
  (intern (string-upcase str) user::*keyword-package*))

(defvar *json-parse-object-as* :alist)
(defvar *json-parse-key-function* #'string->keyword)

(require :json-utils "json-utils.l")

(defparameter *inverse-char-table*
  (plist-hash
   '(#\" #\"
     #\' #\'
     #\\ #\\
     #\/ #\/
     #\b #\Backspace
     #\f #\Page
     #\n #\Newline
     #\r #\Return
     #\t #\Tab)))

(defun space-p (c)
  (member c (list #\Space #\Newline #\Tab #\Linefeed #\Return)))

(defun skip-space (is)
  (prog1 is
    (while (space-p (peek-char is))
      (read-char is))))

(defun peek-char-skipping-space (is)
  (skip-space is)
  (peek-char is))

(defun read-char-skipping-space (is)
  (skip-space is)
  (read-char is))

(defun parse-unicode (is os)
  (write-byte (read-char is) os)
  (write-byte (read-char is) os)
  (write-byte (read-char is) os))

(defun parse-escape-char (is os)
  (let* ((raw-char (read-char is))
         (esc (gethash raw-char *inverse-char-table*)))
    (if esc
        (write-byte esc os)
      (when (lisp::char= raw-char #\u)
        (parse-unicode is os)))))

(defun parse-string (is)
  (with-output-to-string (os)
   (let ((start-symbol (read-char is)))
     ;; assert string begins with ' or "
     (assert (find start-symbol (list #\' #\"))
             (format nil "Error: string '~C~A' must start with ' or \"~%"
                     start-symbol (read-line is nil "")))
     (loop
      (cond
       ((lisp::char= (peek-char is nil #\0) start-symbol) ;; #\0 is dummy
        (read-char is)
        (return-from parse-string
          (get-output-stream-string os)))
       ((lisp::char= (peek-char is nil #\0) #\\) ;; #\0 is dummy
        (read-char is) ;; skip
        (parse-escape-char is os))
       (t (write-byte (read-char is) os)))))))

(defun parse-constant (is)
  (with-output-to-string (os)
    (dotimes (i 4)
      (write-byte (read-char is nil) os))
    (let ((str (get-output-stream-string os)))
      (cond
       ((string= str "true")
        (return-from parse-constant t))
       ((string= str "null")
        (return-from parse-constant nil))
       ((and (peek-char is nil nil)
             (string= "false"
                      (concatenate string str
                       (format nil "~c" (read-char is)))))
        (return-from parse-constant nil))
       (t
        (error "~A is not a constant" str))))))

(defun parse-number (is)
  (with-output-to-string (os)
   (while (position (peek-char is nil nil) ".0123456789+-Ee")
     (write-byte (read-char is) os))
   (let ((num? (read-from-string (get-output-stream-string os))))
     (if (numberp num?) (return-from parse-number num?)
       (error "~A is not a number" num?)))))

(defun parse-array (is)
  (let (ret)
    (read-char is) ;; skip #\[
    (loop
     (cond
      ((lisp::char= (peek-char-skipping-space is) #\])
       (read-char-skipping-space is) ;; skip #\]
       (return-from parse-array (nreverse ret)))
      ((lisp::char= (peek-char-skipping-space is) #\,)
       (read-char-skipping-space is)) ;; skip #\,
      (t
       (push (parse is) ret))))))

(defun add-attribute (to k v)
  (case *json-parse-object-as*
    (:plist
     (append to (list k v)))
    (:alist
     (append to (list (cons k v))))
    (t
     (error "~A is not valid, use :alist or :plist" *json-parse-object-as*))))

(defun convertable-to-date-p (obj)
  (case *json-parse-object-as*
    (:alist
     (string= (string-downcase (string (caar obj))) "$date"))
    (:plist
     (string= (string-downcase (string (car obj))) "$date"))))

(defun datetime-from-object (obj)
  (case *json-parse-object-as*
    (:alist
     (datetime-from-utc-epoch-time (cdar obj)))
    (:plist
     (datetime-from-utc-epoch-time (cadr obj)))))

(defun parse-object (is)
  (let (ret lp)
    (read-char is) ;; skip #\{
    (while (lisp::char/= (peek-char-skipping-space is) #\})
      (setq ret
            (add-attribute ret
             (let ((key-str (parse-string is)))
               (prog1
                   (or (funcall *json-parse-key-function* key-str)
                       (error "cannot convert key ~A" key-str))
                 (unless (eql #\: (read-char-skipping-space is))
                   (error "expected colon not found at key ~A" key-str))
                 (skip-space is)))
             (parse is)))
      (case (peek-char-skipping-space is)
        (#\, (read-char is))
        (#\} nil)))
    (read-char is)
    ret))

(defun parse (is)
  (case (peek-char-skipping-space is)
    ((#\" #\')
     (parse-string is))
    ((#\- #\0 #\1 #\2 #\3 #\4 #\5 #\6 #\7 #\8 #\9)
     (parse-number is))
    (#\{
     (let ((obj (parse-object is)))
       (if (convertable-to-date-p obj)
           (datetime-from-object obj)
         obj)))
    (#\[
     (parse-array is))
    ((#\t #\f #\n)
     (parse-constant is))
    (t
     (error (format nil "Invalid JSON: ~c at ~a" (peek-char is) (read-line is nil ""))))))

(defun parse-from-string (s)
  (let ((is (make-string-input-stream s)))
    (parse is)))

(provide :json-decode) ;; end of json-decode.l
