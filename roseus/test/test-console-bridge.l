#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))
(init-unit-test)

;;
(deftest test-console-bridge
    (dolist (level (list ros::*console-bridge-log-debug*
                         ros::*console-bridge-log-info*
                         ros::*console-bridge-log-warn*
                         ros::*console-bridge-log-error*))
      (warning-message 2 "check ~A level~%" level)
      (ros::console-bridge-set-log-level level)
      (warning-message 2 "  current log level ~A~%" (ros::console-bridge-get-log-level))
      (ros::console-bridge-log-error "this is ERROR message")
      (ros::console-bridge-log-warn "this is WARN message")
      (ros::console-bridge-log-info "this is INFO message")
      (ros::console-bridge-log-debug "this is DEBUG message")
      ))

(run-all-tests)

(exit)
