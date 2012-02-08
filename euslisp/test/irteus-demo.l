(init-unit-test)

(defmacro do-until-key (&rest forms) `(dotimes (i 100) ,@forms))
(deftest demo
  (funcall demo-func)
  )

(run-all-tests)
(exit)
