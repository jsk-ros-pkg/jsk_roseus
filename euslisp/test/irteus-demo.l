(init-unit-test)

(defmacro do-until-key (&rest forms) (with-gensyms (do-until-key-counter) `(dotimes (do-until-key-counter 100) ,@forms)))
(deftest demo
  (funcall demo-func)
  )

(run-all-tests)
(exit)
