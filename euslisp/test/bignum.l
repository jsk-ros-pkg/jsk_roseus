;(setq sys::*gc-hook* #'(lambda (a b) (format t "GC! free:~A  total:~A~%" a b)))
(setq sys::*gc-hook* #'(lambda (a b) ()))

(init-unit-test)
(defmacro check-bignum (op num1 num2 ans)
  `(let (num3)
     (format *error-output* "check (~a #x~x #x~x) -> " (quote ,op) ,num1 ,num2)
     (setq num3 (funcall ,op ,num1 ,num2))
     (format *error-output* "#x~x,  " num3)
     (if (= num3 ,ans)
         (format *error-output* "passed~%")
       (progn
         (warn "failed!! : answer(~x)~%" ,ans)
         (print (list 'ans (class ,ans)
                      'num3 (class num3)))
         ))
     (assert (= num3 ,ans))))

(deftest test-bignum
  (dotimes (i 10)
    (print i)
    (check-bignum #'+ 0 0 0)
    (check-bignum #'+ 1 3 4)
    (check-bignum #'+ -1 -3 -4)

    ;; fixnum op fixnum -> bignum
    (check-bignum #'+ #x18000000 #x18000000 #x30000000)
    (check-bignum #'- (- #x18000000) #x18000000 (- #x30000000))
    (check-bignum #'* #x18000000 2 #x30000000)

    ;;(check-bignum #'/ ?? ?? ??)

    ;; bignum op fixnum -> fixnum
    ;;(check-bignum #'+ ?? ?? ??)
    (check-bignum #'- #x20000000 1 #x1fffffff)
    ;;(check-bignum #'* ?? ?? ??)
    (check-bignum #'/ #x20000000 2 #x10000000)

    ;; fixnum op bignum -> fixnum
    ;;(check-bignum #'+ ?? ?? ??)
    (check-bignum #'- 1 #x20000000 (- #x1fffffff))
    ;;(check-bignum #'* ?? ?? ??)
    (check-bignum #'/ 1 #x20000000 0)

    ;; bignum op bignum -> fixnum
    ;;(check-bignum #'+ ?? ?? ??)
    (check-bignum #'- #x20000001 #x20000000 1)
    ;;(check-bignum #'* ?? ?? ??)
    (check-bignum #'/ #x40000000 #x20000000 2)
    (check-bignum #'/ #x40000000 (- #x20000000) -2)
    (check-bignum #'/ (- #x40000000) #x20000000 -2)
    (check-bignum #'mod #x40000001 #x20000000 1)
    (check-bignum #'mod #x100000001 #x20000000 1)

    ;; bignum op bignum -> bignum
    (check-bignum #'+ #x20000000 #x20000000 #x40000000)
    (check-bignum #'- #x40000000 #x20000000 #x20000000)
    (check-bignum #'* #x20000000 #x20000000 #x400000000000000)
    (check-bignum #'/ #x400000000000000 #x20000000 #x20000000)

    (check-bignum #'mod #x20000000 #x40000000 #x20000000)
    ;;(check-bignum #'mod #x40000000 #x100000000 #x40000000)
    )
  (print 'done)
  )

(run-all-tests)
(exit)

