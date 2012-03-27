(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(dotimes (i 10)
  (sort '(a b) #'string<= #'(lambda(x)(let ((v (instantiate vector 1000)) (r (gc))) (format nil "~A" x))))
  )

(run-all-tests)
(exit)
