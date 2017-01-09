;; json-encode.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(unless (find-package "JSON") (make-package "JSON"))
(in-package "JSON")

(require :json-utils "json-utils.l")

(defparameter *char-table*
  (plist-hash
   '(#\\ "\\\\"
     #\" "\\\""
     #\Backspace "\\b"
     #\Page "\\f"
     #\Newline "\\n"
     #\Return "\\r"
     #\Tab "\\t")))

(defun encode-string (string &optional (os *standard-output*))
  (with-blacket os #\" #\"
    (dotimes (i (length string))
      (let* ((c (aref string i))
             (esc (gethash c *char-table*)))
        (if esc
            (format os esc)
            (write-byte c os))))))


(defun encode-number (num &optional (os *standard-output*))
  (format os "~A" num))

(defun encode-key-value (key value os)
  (encode-element key os)
  (write-byte #\: os)
  (encode-element value os))


(defun encode-vector (v &optional (os *standard-output*))
  (with-blacket os #\[ #\]
    (dotimes (i (length v))
      (encode-number (aref v i))
      (if (< i (1- (length v)))
          (write-byte #\, os)))))


(defun encode-element (elem &optional (os *standard-output*))
  (cond
    ((and (consp elem) (functionp (car elem)))
     (encode-element (eval elem) os))
    ((encodable-plist-p elem)
     (encode-plist elem os))
    ((encodable-alist-p elem)
     (encode-alist elem os))
    ((consp elem)
     (encode-list elem os))
    ((stringp elem)
     (encode-string elem os))
    ((numberp elem)
     (encode-number elem os))
    ((vectorp elem)
     (encode-vector elem os))
    ((keywordp elem)
     (encode-string (string-downcase (string elem)) os))
    ((null elem)
     (format os "false"))
    ((eq t elem)
     (format os "true"))
    ((derivedp elem calendar-time)
     (encode-time (eval elem) os))
    ((derivedp elem ros::time)
     (encode-ros-time (eval elem) os))
    (t
     (encode-element (eval elem) os))))

(defun encode-time (ctime &optional (os *standard-output*))
  (with-blacket os #\{ #\}
    (encode-key-value "$date" (get-utc-epoch-time ctime) os)))

(defun encode-ros-time (rtime &optional (os *standard-output*))
  (with-blacket os #\{ #\}
    (encode-key-value "$date"
                      (+ (* 1000 (send rtime :sec))
                         (/ (send rtime :nsec) 1000 1000)) os)))

(defun encode-list (l &optional (os *standard-output*))
  (with-blacket os #\[ #\]
    (labels ((r-encode-list (l s)
               (encode-element (car l) s)
               (when (cdr l)
                 (write-byte #\, s)
                 (r-encode-list (cdr l) s))))
      (r-encode-list l os))))

(defun encodable-plist-p (plist)
  (and (consp plist) (keywordp (car plist))))

(defun encodable-alist-p (alist)
  (if (and (consp alist) (consp (car alist)))
      (keywordp (caar alist))))

(defun encode-assoc-key-value (key value os)
  (let ((s (string-downcase (string key))))
    (encode-key-value s value os)))

(defun encode-alist (alist &optional (os *standard-output*))
 (with-blacket os #\{ #\}
    (labels ((r-encode-alist (kv l s)
               (if (consp (cdr kv))
                   (progn
                     (format os "\"~A\":" (string-downcase (string (car kv))))
                     (encode-element (cdr kv) os))
                   (encode-assoc-key-value (car kv) (cdr kv) os))
               (when l
                 (write-byte #\, os)
                 (r-encode-alist (car l) (cdr l) s))))
      (r-encode-alist (car alist) (cdr alist) os)))
)

(defun encode-plist (plist &optional (os *standard-output*))
  (with-blacket os #\{ #\}
    (labels ((r-encode-plist (pl s)
               (cond
                 ((<= 2 (length pl))
                  (if (consp (cadr pl))
                      (progn
                        (format os "\"~A\":" (string-downcase (string (car pl))))
                        (encode-element (cadr pl) os))
                      (encode-assoc-key-value (car pl) (cadr pl) os)))
                 ((eq 1 (length pl))
                  (encode-key-value (car pl) nil os)))
               (when (cddr pl)
                 (write-byte #\, os)
                 (r-encode-plist (cddr pl) s))))
      (r-encode-plist plist os))))

(defun stream->string (encoder elem)
  (let ((ss (make-string-output-stream 4096)))
    (funcall encoder elem ss)
    (get-output-stream-string ss)))

(provide :json-encode) ;; end of json-encode.l
