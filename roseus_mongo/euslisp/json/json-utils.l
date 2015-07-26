;; json-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defun plist-hash (plist &optional (hash (make-hash-table)))
  (if (null plist) (return-from plist-hash hash))
  (assert (evenp (length plist)) "plist must be pairs")
  (sethash (car plist) hash (cadr plist))
  (plist-hash (cddr plist) hash))

(defmacro with-blacket (stream opening-char closing-char &rest body)
  `(progn
     (write-byte ,opening-char ,stream)
     ,@body
     (write-byte ,closing-char ,stream)))

(defun get-epoch-time (ctime)
  (+
   (* 1000 (send ctime :compute-seconds))
   (/ (send ctime :micro) 1000)))

(defun datetime-from-epoch-time (etime)
  (instance calendar-time :init
            (/ etime 1000)
            (mod etime 1000)))


(provide :json-utils) ;; end of json-utils.l
