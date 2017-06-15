#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib")
(ros::load-ros-manifest "roscpp")

(init-unit-test)
;;;
;;;

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(defclass simple-object
  :super object
  :slots ())
(defmethod simple-object
  (:init ()  (sys::dispose-hook self t) self)
  (:dispose () (format *error-output* "dispose ~A~%" self) (setq *dispose* (+ *dispose* 1)))
  )

(deftest test-simple-object ()
  (warning-message 1 "check dispose")
  (setq *dispose* 0)
  (dotimes (i 4)
    (print (instance simple-object :init)))

  (sys::gc) ;; dipose
  (warning-message 1 "dispose = ~A~%" *dispose*)
  (assert (>= *dispose* 2)))

(unless (find-package "ROSACTIONLIB") (make-package "ROSACTIONLIB"))
(defclass gentemp-object
  :super object
  :slots ())
(defmethod gentemp-object
  (:init ()  (sys::dispose-hook self t) (set (gentemp "ACTIONLIBCLIENT" (find-package "ROSACTIONLIB")) self) self)
  (:dispose () (format *error-output* "dispose ~A~%" self) (setq *dispose* (+ *dispose* 1)))
  )

(deftest test-gentemp-object ()
  (warning-message 1 "check gentemp")
  (setq *dispose* 0)
  (dotimes (i 4)
    (print (instance gentemp-object :init)))

  (sys::gc) ;; not dipose
  (warning-message 1 "dispose = ~A~%" *dispose*)
  ;;;(assert (>= *dispose* 2)) ;; this is expected
  )

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "test")
(defun cb (msg) (print msg))
(defclass cbfunc-object
  :super object
  :slots ())
(defmethod cbfunc-object
  (:init ()
         (ros::subscribe "listener" std_msgs::Bool #'cb)
         (sys::dispose-hook self t)
         self)
  (:cb (msg) (print msg))
  (:dispose () (format *error-output* "dispose ~A~%" self) (setq *dispose* (+ *dispose* 1)))
  )

(deftest test-calback-func-object ()
  (setq *dispose* 0)
  (warning-message 1 "check callback func")
  (dotimes (i 4)
    (print (instance cbfunc-object :init)))

  (sys::gc) ;; dipose
  (warning-message 1 "dispose = ~A~%" *dispose*)
  (assert (>= *dispose* 2)))

(defclass methodcb-object
  :super object
  :slots ())
(defmethod methodcb-object
  (:init ()
         (ros::subscribe "listener" std_msgs::Bool #'send self :cb)
         (sys::dispose-hook self t)
         self)
  (:cb (msg) (print msg))
  (:dispose () (format *error-output* "dispose ~A~%" self) (setq *dispose* (+ *dispose* 1)))
  )

(deftest test-calback-method-object ()
  (setq *dispose* 0)
  (warning-message 1 "check self callback")
  (dotimes (i 4)
    (print (instance methodcb-object :init)))

  (sys::gc) ;; not dipose  : fixed by #524
  (warning-message 1 "dispose = ~A~%" *dispose*)
  (assert (>= *dispose* 2)))


;(ros::service-call "~set_logger_level"
;                   (instance roscpp::SetLoggerLevelRequest
;                             :init :logger "ros.roseus" :level "DEBUG"))

(run-all-tests)
(exit)
