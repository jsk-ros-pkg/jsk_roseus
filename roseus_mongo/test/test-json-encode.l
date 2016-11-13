;; test-json-encode.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :json-encode "package://roseus_mongo/euslisp/json/json-encode.l")

(setq adate (send (instance calendar-time :init) :make :year 1991 :month 2 :day 24))

(init-unit-test)

(deftest test-timezone ()
  (setq a-date-1 (now))
  (assert (eq (send a-date-1 :compute-seconds)
              (send (json::datetime-from-utc-epoch-time (json::get-utc-epoch-time a-date-1)) :compute-seconds))))

(setq json::*tzoffset* 0)
(deftest test-encode-alist ()
  (warn "~%(json::encode-alist '((:a . 1)))~%")
  (assert (string= "{\"a\":1}" (json::stream->string #'json::encode-alist '((:a . 1)))))

  (warn "~%(json::encode-alist '((:a . t)))~%")
  (assert (string= "{\"a\":true}" (json::stream->string #'json::encode-alist '((:a . t)))))
  (warn "~%(json::encode-alist '((:a . nil)))~%")
  (assert (string= "{\"a\":false}" (json::stream->string #'json::encode-alist '((:a . nil)))))

  (setq adate (send (instance calendar-time :init) :make :year 1991 :month 2 :day 24))
  (warn "~%(json::encode-alist '((:a . adate)))~%")
  (assert (string= "{\"a\":{\"$date\":667353600000}}"
                   (json::stream->string #'json::encode-alist '((:a . adate)))))

  (assert (string= "{\"a\":{\"b\":{\"$date\":667353600000}}}"
                   (json::stream->string #'json::encode-alist
                                         '((:a .
                                            ((:b . (send (instance calendar-time :init) :make :year 1991 :month 2 :day 24))))))))

  (warn "~%(json::encode-alist '((:a . rtime)))~%")
  (setq rtime (ros::time 100.5))
  (assert (string= "{\"a\":{\"$date\":100500}}"
                   (json::stream->string #'json::encode-alist '((:a . rtime)))))

  (warn "~%(json::encode-alist '((:a . (+ 1 2 3))))~%")
  (assert (string= "{\"a\":6}" (json::stream->string #'json::encode-alist '((:a . (+ 1 2 3))))))

  (warn "~%(json::encode-alist '((:a 1 2 3 4)))~%")
  (assert (string= "{\"a\":[1,2,3,4]}" (json::stream->string #'json::encode-alist '((:a 1 2 3 4)))))

  (warn "(json::encode-alist '((:a . 1) (:b . 2)))~%")
  (assert (string= "{\"a\":1,\"b\":2}" (json::stream->string #'json::encode-alist '((:a . 1) (:b . 2)))))

  (warn "(json::encode-alist '((:a . ((:b . 2)))))~%")
  (assert (string= "{\"a\":{\"b\":2}}" (json::stream->string #'json::encode-alist '((:a . ((:b . 2)))))))

  (warn "(json::encode-alist '((:a . ((:b . 2) (:c . 3)))))~%")
  (assert (string= "{\"a\":{\"b\":2,\"c\":3}}" (json::stream->string #'json::encode-alist '((:a . ((:b . 2) (:c . 3)))))))
  )


(deftest test-encode-plist ()
  (warn "~%(json::encode-plist '(:a 1))~%")
  (assert (string= "{\"a\":1}" (json::stream->string #'json::encode-plist '(:a 1))))

  (assert (string= "{\"a\":true}" (json::stream->string #'json::encode-plist '(:a t))))
  (assert (string= "{\"a\":false}" (json::stream->string #'json::encode-plist '(:a nil))))

  (setq adate (send (instance calendar-time :init) :make :year 1991 :month 2 :day 24))
  (assert (string= "{\"a\":{\"$date\":667353600000}}"
                   (json::stream->string #'json::encode-plist '(:a adate))))

  (setq rtime (ros::time 100.5))
  (assert (string= "{\"a\":{\"$date\":100500}}"
                   (json::stream->string #'json::encode-plist '(:a rtime))))

  (assert (string= "{\"a\":6}" (json::stream->string #'json::encode-plist '(:a (+ 1 2 3)))))

  (warn "~%(json::encode-plist '(:a (1 2 3 4)))~%")
  (assert (string= "{\"a\":[1,2,3,4]}" (json::stream->string #'json::encode-plist '(:a (1 2 3 4)))))

  (warn "(json::encode-plist '(:a 1 :b 2))~%")
  (assert (string= "{\"a\":1,\"b\":2}" (json::stream->string #'json::encode-plist '(:a 1 :b 2))))

  (warn "(json::encode-plist '(:a (:b 2)))~%")
  (assert (string= "{\"a\":{\"b\":2}}" (json::stream->string #'json::encode-plist '(:a (:b 2)))))

  (warn "(json::encode-plist '(:a (:b 2 :c 3)))~%")
  (assert (string= "{\"a\":{\"b\":2,\"c\":3}}" (json::stream->string #'json::encode-plist '(:a (:b 2 :c 3)))))
  )

(run-all-tests)
(exit)
;; end of test-json-encode.l
