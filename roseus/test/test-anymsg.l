#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(ros::load-ros-manifest "roseus")

(init-unit-test)

;; https://answers.ros.org/question/202708/rospymsganymsg-how-do-we-use/?answer=286336#post-id-286336
(deftest test-anymsg ()
  (let (msg msgclass)
    (ros::roseus "test_anymsg")
    ;;
    (ros::subscribe "chatter" ros::anymsg #'(lambda (m) (setq msg m)))
    ;;
    (ros::rate 10)
    (while (and (ros::ok) (null msg))
      (ros::sleep)
      (ros::spin-once))
    (ros::ros-info "received any msg ~A" msg)
    (assert msg "msg ~A" msg)
    ;; restore msg if you aldready know the message calss (std_msgs::String)
    (setq m (instance std_msgs::String :init))
    (send m :deserialize (send msg :buf))
    (ros::ros-info "convert anymsg to std_msgs::string ~A" m)
    (ros::ros-info "  check the :data of string ~A" (send m :data))
    (assert (substringp "hello world" (send m :data)) "msg ~A" m)
    ;; restore msg by retrieving message class from data type
    (ros::ros-info "info for anymsg md5sum ~A, datatype ~A" (send msg :md5sum-) (send msg :datatype-))
    (ros::ros-info "info for anymsg md5sum ~A, datatype ~A from connection header"
                   (cdr (assoc "type" (send msg :connection-header) :test #'string=))
                   (cdr (assoc "md5sum" (send msg :connection-header) :test #'string=)))
    (let* ((type (cdr (assoc "type" (send msg :connection-header) :test #'string=)))
           (pos (position #\/ type)))
      (setq msgcls (eval (intern (string-upcase (subseq type (1+ pos))) (string-upcase (subseq type 0 pos))))))
    (cond ((string= (get msgcls :md5sum-) (cdr (assoc "md5sum" (send msg :connection-header) :test #'string=)))
           (setq m (instance msgcls :init))
           (send m :deserialize (send msg :buf))
           (assert (substringp "hello world" (send m :data)) "msg ~A" m)
           )
          (t
           (ros::ros-error "md5sum of received anymsg and current workspace differ ~A vs ~A"
                           (cdr (assoc "md5sum" (send msg :connection-header) :test #'string=)) (get msgcls :md5sum-))
           (ros::ros-error "callerid ~A, definition ~A"
                           (assoc "callerid" (send msg :connection-header) :test #'string=)
                           (assoc "message_definition" (send msg :connection-header) :test #'string=))))
  ))

(run-all-tests)

(exit)

