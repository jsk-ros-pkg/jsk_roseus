#!/usr/bin/env roseus

(load "package://roseus/euslisp/dynamic-reconfigure-server.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "dynamic_reconfigure")

(ros::roseus "simple_dynamic_reconfigure_server" :anonymous nil)

(setq *int-param* nil)
(setq *double-param* nil)
(setq *str-param* nil)
(setq *bool-param* nil)

(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("int_param"  int_t  1  "Int parameter" 0  -10  10)
         ("double_param"  double_t  2  "double parameter" 0.0  -2.0  10.0)
         ("str_param"  str_t  4  "String parameter" "foo")
         ("bool_param"  bool_t  8  "Boolean parameter"  nil))
        ;;; callbackfunc (defun func (config level) ) -> return updated-config
        '(lambda-closure nil 0 0 (cfg level)
           (setq *updated* t)
           (setq *int-param* (cdr (assoc "int_param" cfg :test #'equal)))
           (setq *double-param* (cdr (assoc "double_param" cfg :test #'equal)))
           (setq *str-param* (cdr (assoc "str_param" cfg :test #'equal)))
           (setq *bool-param* (cdr (assoc "bool_param" cfg :test #'equal)))
           cfg)))

(ros::advertise "~/int_param" std_msgs::Int16)
(ros::advertise "~/double_param" std_msgs::Float32)
(ros::advertise "~/str_param" std_msgs::String)
(ros::advertise "~/bool_param" std_msgs::Bool)
(ros::advertise "~/updated" std_msgs::Bool)

(setq *updated* nil)

(ros::rate 10)
(while (ros::ok)
  (if *int-param*
    (ros::publish "~/int_param" (instance std_msgs::Int16 :data (round *int-param*))))
  (if *double-param*
    (ros::publish "~/double_param" (instance std_msgs::Float32 :data *double-param*)))
  (if *str-param*
    (ros::publish "~/str_param" (instance std_msgs::String :data *str-param*)))
  (ros::publish "~/bool_param" (instance std_msgs::Bool :data *bool-param*))
  (ros::publish "~/updated" (instance std_msgs::Bool :data *updated*))
  (ros::spin-once)
  )
