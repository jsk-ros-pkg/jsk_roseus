;; test-json-decode.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :json-decode "package://roseus_mongo/euslisp/json/json-decode.l")

(setq json::*tzoffset* 0)

(init-unit-test)

(deftest test-skip-space ()
  (assert (equal #\0
           (read-char (with-input-from-string (is "   0")
                       (json::skip-space is))))))

(deftest test-read-char ()
  (assert (equal #\a
           (with-input-from-string (is " abcdef")
            (json::peek-char-skipping-space is))))
  (assert (equal (list #\a #\b #\c)
           (with-input-from-string (is " ab   cdef")
            (list
             (json::read-char-skipping-space is)
             (json::read-char-skipping-space is)
             (json::read-char-skipping-space is)
             )))))

(deftest test-json-decode-number ()
  (assert (equal 123456 (json::parse-from-string "123456")))
  (assert (equal -123456 (json::parse-from-string "-123456")))
  (assert (equal -123456.0345 (json::parse-from-string "-123456.0345")))
  (assert (equal -123456.0345e+5 (json::parse-from-string "-123456.0345e+5")))
  )

(deftest test-json-decode-constant ()
  (assert (eq t (json::parse-from-string "true")))
  (assert (eq t (json::parse-from-string "   true")))
  (assert (eq t (json::parse-from-string "   true    ")))
  (assert (eq nil (json::parse-from-string "false")))
  (assert (eq nil (json::parse-from-string "null")))
  )

(deftest test-json-decode-string ()
  (assert (string= "fuga" (json::parse-from-string "\"fuga\"")))
  (assert (string= "    fuga  " (json::parse-from-string "\"    fuga  \"")))
  (assert (string= "\t\r\n\/\\ fuga" (json::parse-from-string "\"\t\r\n\/\\\\ fuga\"")))
  (assert (string= "ほげ" (json::parse-from-string "\"ほげ\"")))
  (assert (string= "{\"Samples\": [1]}" (json::parse-from-string "\"{\\\"Samples\\\": [1]}\"")))
  )

(deftest test-json-decode-array ()
  (assert (eq
           nil
           (json::parse-from-string "[]")))
  (assert (equal
           (list "fuga" 1 t)
           (json::parse-from-string "[\"fuga\",1,true]")))
  (assert (equal
           (list "fuga" 1 nil)
           (json::parse-from-string "[\"fuga\"    ,1,false]")))
  (assert (equal
           (list "fuga" 1 nil)
           (json::parse-from-string "[ \"fuga\"    ,1 ,   null]")))
  )

(deftest test-json-decode-object ()
  (assert (equal
           '((:1 . 2))
           (with-input-from-string (is "{'1':2}")
                         (json::parse-object is))))
  (assert (equal
           '((:1 . "abc"))
           (with-input-from-string (is "{'1':\"abc\"}")
                         (json::parse-object is))))
  (assert (equal
           '((:1 . 2) (:2 . t))
           (with-input-from-string (is "{\"1\":2,  \"2\": true}")
                         (json::parse-object is))))
  (assert (equal
           '((:1 (:abc . 2)))
           (with-input-from-string (is "{'1':{\"abc\":2}}")
                         (json::parse-object is))))
  (assert (equal
           '((:1 (:abc 2 t)))
           (with-input-from-string (is "{'1':{\"abc\":[2,true]}}")
                         (json::parse-object is))))
  (assert (equal
           '((:1 ((:2 . 3)) ((:4 . t))))
           (with-input-from-string (is "{'1':[{'2':3},{'4':true}]}")
                         (json::parse-object is))))
  )

(deftest test-json-decode-date ()
  (setq json::*tzoffset* 0)
  (setq res1 (json::parse-from-string "{'$date': 1437900513823}"))
  (assert (and
           (eq (send res1 :year) 2015)
           (eq (send res1 :month) 7)
           (eq (send res1 :day) 26)
           (eq (send res1 :hour) 8)
           (eq (send res1 :minute) 48)
           (eq (send res1 :second) 33)
           (eq (send res1 :micro) 823)))

  (setq json::*tzoffset* 9)
  (setq res2 (json::parse-from-string "{'$date': 1437900513823}"))
  (assert (and
           (eq (send res2 :year) 2015)
           (eq (send res2 :month) 7)
           (eq (send res2 :day) 26)
           (eq (send res2 :hour) 17)
           (eq (send res2 :minute) 48)
           (eq (send res2 :second) 33)
           (eq (send res2 :micro) 823))))

(run-all-tests)
(exit)
;; end of test-json-decode.l
