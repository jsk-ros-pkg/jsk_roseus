#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-compile-message")

;(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(deftest test-compile-message ()
  (ros::roseus-add-msgs "roseus")
  (assert
   (boundp (read-from-string "roseus::string"))
   "fail to load roseus::string message")
  (assert
   (not (derivedp (cadr (assoc :init (send (eval (read-from-string "roseus::string")) :methods))) compiled-code))
   "loading compiled code while loading scripts")
  (setq ros::*compile-message* t)
  (ros::roseus-add-msgs "roseus")
  (assert
   (derivedp (cadr (assoc :init (send (eval (read-from-string "roseus::string")) :methods))) compiled-code)
   "fail to load compiled code")
  )

(run-all-tests)

(exit)
