#!/usr/bin/env roseus

;;
;; $ rostopic pub -r 4 /a std_msgs/String 'from a'
;; $ rostopic pub -r 4 /b std_msgs/String 'from b'
;;
(require :unittest "lib/llib/unittest.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "test-multi-queue")
(ros::create-nodehandle "spinb")

(defun callback-a (msg)
  (print (list 'a (send msg :data)))
  (setq *subscribed-string* (send msg :data)))
(defun callback-b (msg)
  (print (list 'b (send msg :data)))
  (setq *subscribed-string* (send msg :data)))

(ros::subscribe "/a" std_msgs::String #'callback-a)
(ros::subscribe "/b" std_msgs::String #'callback-b :groupname "spinb")

(init-unit-test)

(deftest test-common ()
  (unix:sleep 3)
  (setq *success-a* nil)
  (setq *success-b* nil)
  (dotimes (i 10)
    (setq *subscribed-string* nil)
    (ros::spin-once)
    (when *subscribed-string*
      (assert (string= *subscribed-string* "from a"))
      (setq *success-a* t))

    (setq *subscribed-string* nil)
    (ros::spin-once "spinb")
    (when *subscribed-string*
      (assert (string= *subscribed-string* "from b"))
      (setq *success-b* t))
    (unix::usleep (* 500 1000)))
  (assert
   (and *success-a* *success-b*))
  )

(run-all-tests)

(exit)
