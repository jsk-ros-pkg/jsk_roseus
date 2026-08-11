#!/usr/bin/env roseus
;;;
;;;
(ros::load-ros-manifest "roseus")
(require :unittest "lib/llib/unittest.l")

(init-unit-test)


(ros::roseus "sub_double")
;; method call
(defclass double-cb-class
  :super propertied-object
  :slots (count prev-msg))
(defmethod double-cb-class
  (:init
   ()
   (setq count 0)
   (setq prev-msg nil)
   (ros::subscribe "double" std_msgs::float64 #'send self :double-cb))
  (:double-cb
   (msg)
   (warning-message 2 "subscribe msg [~8,8f], ~A times bigger than previous data~%" (send msg :data) (if prev-msg (/ (send msg :data) (send prev-msg :data))))
   (when prev-msg
     (assert (eps= (/ (send msg :data) (send prev-msg :data)) 2.0))
     )
   (incf count)
   (setq prev-msg msg)
   )
  (:count () count)
  )

(deftest test-sub-double ()
  (setq m (instance double-cb-class :init))

  (while (< (send m :count) 10)
    (ros::spin-once)
    ))

(run-all-tests)
(exit)
