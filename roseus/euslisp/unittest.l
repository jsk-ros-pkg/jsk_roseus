;;;
;;; based on euslib/jsk/unittest.l by R.Ueda
;;;

(defvar *unit-test* nil)

(defun unittest-error (code msg1 form &optional (msg2))
  (format *error-output* "~C[1;3~Cm~A unittest-error: ~A"
	  #x1b (+ 1 48)   *program-name* msg1)
  (if msg2 (format *error-output* " ~A" msg2))
  (if form (format *error-output* " in ~s" form))
  (format *error-output* ", exitting...~C[0m~%"  #x1b)
  (send *unit-test* :increment-failure (format nil "~A" form) msg1)
  (send *unit-test* :print-result)
  (exit 1))

(defun unittest-sigint-handler (sig code)
  (format *error-output* "unittest-sigint-handler ~A~%" sig)
  (send *unit-test* :increment-failure "" (format nil "signal-handler ~A" sig))
  (send *unit-test* :print-result)
  (exit 1))

(defclass unit-test-result
  :super propertied-object
  :slots (name tests failures errors time))
(defmethod unit-test-result
  (:init (n) (setq name n tests 0 failures 0 errors 0 time 0))
  (:tests () tests)
  (:failures () failures)
  (:successes () (- tests failures))
  (:time () time)
  (:prin1
   (strm)
   (format strm "RESULT: ~A~%" name)
   (format strm "  TEST-NUM: ~A~%" (send self :tests))
   (format strm "    PASSED:   ~A~%" (send self :successes))
   (format strm "    FAILURE:  ~A~%" (send self :failures))
   )
  (:prin1-xml
   (strm)
   (format strm "  <testcase name=\"~A\">~%" name)
   (dotimes (i (send self :failures))
     (format strm "   <failure type=\"AssertionError\">[WARN]")
     (format strm "   </failure>~%"))
   (format strm "  </testcase>~%")
   )
  )

(defclass unit-test-container
  :super propertied-object
  :slots (result functions log-fname output-mode))

(defmethod unit-test-container
  (:init
   (&key ((:log-fname fname)))
   (setq result nil)
   (setq functions nil)
   (setq log-fname fname)
   (when log-fname
     (warning-message 3 "output to ~A~%" log-fname)
     (setq output-mode :xml))
   self)
  (:functions () (reverse functions))
  ;;
  (:add-function (name) (push name functions))
  (:increment-tests
   ()
   (if result (incf (unit-test-result-tests (car result)))))
  (:increment-failure
   (test str)
   (if result (incf (unit-test-result-failures (car result))))
   (format *error-output* "~C[3~Cm[WARN] test ~A failed" #x1b 49 test)
   (if str (format *error-output* " ... (~A)" str))
   (format *error-output* ".~C[0m~%" #x1b))
  (:set-time-to-current-result
   (time)
   (if result (setf (unit-test-result-time (car result)) time)))
  (:init-result
   (func-sym)
   (push (instance unit-test-result :init func-sym) result))
  (:clear-result () (setq result nil))
  ;;
  (:print-xml-result
   (&optional (strm t))
   (let ((all-tests (apply #'+ (send-all result :tests)))
         (all-times (apply #'+ (send-all result :time)))
         (all-failures (apply #'+ (send-all result :failures))))
     (format strm "<testsuite name=\"AllTests\" tests=\"~A\" time=\"~0,3f\" failures=\"~A\" errors=\"~A\">~%" all-tests all-times all-failures 0)
     (dolist (r (reverse result)) (send r :prin1-xml strm))
     (format strm "</testsuite>~%")
     ))
  (:print-normal-result
   (&optional (strm t))
   (let ((all-tests (apply #'+ (send-all result :tests)))
         (all-successes (apply #'+ (send-all result :successes)))
         (all-failures (apply #'+ (send-all result :failures))))
     (format strm "ALL RESULTS:~%")
     (format strm "  TEST-NUM: ~A~%" all-tests)
     (format strm "    PASSED:   ~A~%" all-successes)
     (if (> all-failures 0)
         (format strm "~C[3~Cm" #x1b 49))
     (format strm "    FAILURE:  ~A~%" all-failures)
     (if (> all-failures 0)
         (format strm "~C[0m" #x1b))
     (dolist (r (reverse result)) (prin1 r strm))
     ))
  (:print-result
   ()
   (send self :print-normal-result)
   (when (and result (eq output-mode :xml))
     (let ((strm (open log-fname :direction :output
		       :if-exists :append
		       :if-does-not-exist :create)))
       (send self :print-xml-result strm)
       (close strm))))
  )

(defmacro deftest (name &rest body)
  `(progn
     ;; its not cool...
     (defun ,name ()
       ,@body)
     (send *unit-test* :add-function ',name)
     ',name))

(defmacro assert (pred &optional (message "") &rest args)
  `(progn
     (send *unit-test* :increment-tests)
     (print (list ',pred ,message ,@args))
     (if (not ,pred)
       (send *unit-test* :increment-failure ',pred (format nil ,message ,@args)))))

(defun run-test (func-sym)
  ;; initilize result
  (send *unit-test* :init-result func-sym)
  (format t "TEST-NAME: ~A~%" func-sym)
  (format t "  now testing...~%")
  (let ((func (symbol-function func-sym))
        (tm (instance mtimer :init)))
    (send tm :start)
    (let ((r (funcall func)))
      (let ((time (send tm :stop)))
        (send *unit-test* :set-time-to-current-result time)))))

(defun run-all-tests ()
  ;; initalize *unit-test-result*
  (send *unit-test* :clear-result)
  (dolist (u (send *unit-test* :functions))
    (run-test u))
  (send *unit-test* :print-result)
  t)

(defun init-unit-test (&key log-fname)
  (let* ((p "--gtest_output=xml:")
         (s (find-if #'(lambda (x) (substringp p x)) lisp::*eustop-argument*))
         (xml-fname (if s (string-left-trim p s))))
    (if xml-fname (setq log-fname xml-fname))

    (setq lisp::*exit-on-fatal-error* t)
    (lisp::install-error-handler 'unittest-error)
    (unix:signal unix::sigint 'unittest-sigint-handler)
    (unix:signal unix::sighup 'unittest-sigint-handler)

    (setq *unit-test* (instance unit-test-container :init :log-fname log-fname))
    t))

(provide :gtest)
