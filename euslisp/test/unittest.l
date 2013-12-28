;; test code for unittest

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest test1
  )

(deftest test2
  (dotimes (i 10)
    (assert t "true"))
  )

(deftest test3
  (dotimes (i 10)
    (assert (null nil) "false"))
  )

(deftest test4
  (dotimes (i 10)
    (assert t "ok"))
  (dotimes (i 10)
    (assert (null nil) "false"))
  (dotimes (i 10)
    (assert t "ok"))
  )

(run-all-tests)
(exit)
