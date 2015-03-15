;; remname __name for test (we can not remove this, whcih causes error on eustop in toplevel.l
(setq lisp::*eustop-argument* (substitute-if "__dummy_name:=name" #'(lambda (s) (string= (subseq s 0 8) "__name:=")) lisp::*eustop-argument*))
;;
(ros::roseus "test_anonymous" :anonymous t)
(ros::advertise "chatter" std_msgs::string 1)
(ros::rate 10)
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
  (ros::publish "chatter" msg)
  (ros::sleep)
  )



