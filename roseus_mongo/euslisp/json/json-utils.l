;; json-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(unless (find-package "JSON") (make-package "JSON"))
(in-package "JSON")

(defvar *tzoffset* 9) ;; japanese standard time is 9 hours earlier than UTC.

(defun plist-hash (plist &optional (hash (make-hash-table)))
  (if (null plist) (return-from plist-hash hash))
  (assert (evenp (length plist)) "plist must be pairs")
  (sethash (car plist) hash (cadr plist))
  (plist-hash (cddr plist) hash))

(defmacro with-blacket (stream opening-char closing-char &rest body)
  (user::with-gensyms (ret)
    `(let (,ret)
       (write-byte ,opening-char ,stream)
       (setq ,ret (progn ,@body))
       (write-byte ,closing-char ,stream)
       ,ret)))

(defun get-utc-epoch-time (ctime &key (tzoffset *tzoffset*))
  (let ((utc-time (send ctime :subtract (instance interval-time :make :hour tzoffset))))
    (+
     (* 1000 (send utc-time :compute-seconds))
     (/ (send utc-time :micro) 1000))))

(defun datetime-from-utc-epoch-time (etime &key (tzoffset *tzoffset*))
  (send
   (instance calendar-time :init
             (/ etime 1000)
             (mod etime 1000))
   :add (instance interval-time :make :hour tzoffset)))

(provide :json-utils) ;; end of json-utils.l
