#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
(load "package://roseus/euslisp/roseus-utils.l")

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "test_dynamic_reconfigure_client" :anonymous nil)

(init-unit-test)


(deftest test-dynamic-reconfigure-client-default ()
  (let* ((server-name "/simple_dynamic_reconfigure_server")
         (int-topic-name (format nil "~A/int_param" server-name))
         (double-topic-name (format nil "~A/double_param" server-name))
         (str-topic-name (format nil "~A/str_param" server-name))
         (bool-topic-name (format nil "~A/bool_param" server-name))
         (int-msg nil)
         (double-msg nil)
         (str-msg nil)
         (bool-msg nil)
         (cnt 0))
    (ros::subscribe int-topic-name std_msgs::Int16
                    #'(lambda (m) (setq int-msg m)))
    (ros::subscribe double-topic-name std_msgs::Float32
                    #'(lambda (m) (setq double-msg m)))
    (ros::subscribe str-topic-name std_msgs::String
                    #'(lambda (m) (setq str-msg m)))
    (ros::subscribe bool-topic-name std_msgs::Bool
                    #'(lambda (m) (setq bool-msg m)))

    (while (and (ros::ok)
                (< cnt 100)
                (or (null int-msg) (null double-msg)
                    (null str-msg) (null bool-msg))
                )
      (incf cnt)
      (ros::spin-once)
      (ros::sleep))

    (ros::unadvertise int-topic-name)
    (ros::unadvertise double-topic-name)
    (ros::unadvertise str-topic-name)
    (ros::unadvertise bool-topic-name)

    (pprint double-msg)
    (assert (and int-msg (equal (send int-msg :data) 0))
            (format nil "int default value is wrong: ~A" (send int-msg :data)))
    (assert (and double-msg (eps= (send double-msg :data) 0.0))
            (format nil "double default value is wrong: ~A" (send double-msg :data)))
    (assert (and str-msg (equal (send str-msg :data) "foo"))
            (format nil "str default value is wrong: ~A" (send str-msg :data)))
    (assert (and bool-msg (equal (send bool-msg :data) nil))
            (format nil "bool default value is wrong: ~A" (send bool-msg :data)))
    ))

(deftest test-dynamic-reconfigure-client-update ()
  (let* ((server-name "/simple_dynamic_reconfigure_server")
         (int-topic-name (format nil "~A/int_param" server-name))
         (double-topic-name (format nil "~A/double_param" server-name))
         (str-topic-name (format nil "~A/str_param" server-name))
         (bool-topic-name (format nil "~A/bool_param" server-name))
         (update-topic-name (format nil "~A/updated" server-name))
         (int-msg nil)
         (double-msg nil)
         (str-msg nil)
         (bool-msg nil)
         (update-msg nil)
         (cnt 0))
    (ros::set-dynamic-reconfigure-param server-name "int_param" :int 8)
    (ros::set-dynamic-reconfigure-param server-name "double_param" :double 7.3)
    (ros::set-dynamic-reconfigure-param server-name "str_param" :string "test")
    (ros::set-dynamic-reconfigure-param server-name "bool_param" :bool t)

    (ros::subscribe update-topic-name std_msgs::Bool
                    #'(lambda (m) (setq update-msg m)))
    (while (and (ros::ok) (or (null update-msg) (null (send update-msg :data))))
      (incf cnt)
      (ros::spin-once)
      (ros::sleep))
    (ros::unsubscribe update-topic-name)

    (ros::subscribe int-topic-name std_msgs::Int16
                    #'(lambda (m) (setq int-msg m)))
    (ros::subscribe double-topic-name std_msgs::Float32
                    #'(lambda (m) (setq double-msg m)))
    (ros::subscribe str-topic-name std_msgs::String
                    #'(lambda (m) (setq str-msg m)))
    (ros::subscribe bool-topic-name std_msgs::Bool
                    #'(lambda (m) (setq bool-msg m)))

    (while (and (ros::ok)
                (< cnt 100)
                (or (null int-msg) (null double-msg)
                    (null str-msg) (null bool-msg))
                )
      (incf cnt)
      (ros::spin-once)
      (ros::sleep))

    (ros::unsubscribe int-topic-name)
    (ros::unsubscribe double-topic-name)
    (ros::unsubscribe str-topic-name)
    (ros::unsubscribe bool-topic-name)

    (pprint double-msg)
    (assert (and int-msg (equal (send int-msg :data) 8))
            (format nil "int default value is wrong: ~A" (send int-msg :data)))
    (assert (and double-msg (eps= (send double-msg :data) 7.3))
            (format nil "double default value is wrong: ~A" (send double-msg :data)))
    (assert (and str-msg (equal (send str-msg :data) "test"))
            (format nil "str default value is wrong: ~A" (send str-msg :data)))
    (assert (and bool-msg (equal (send bool-msg :data) t))
            (format nil "bool default value is wrong: ~A" (send bool-msg :data)))
    ))

(run-all-tests)

(exit)
