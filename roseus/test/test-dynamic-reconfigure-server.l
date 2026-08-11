#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")
(load "package://roseus/euslisp/dynamic-reconfigure-server.l")

(ros::roseus-add-msgs "dynamic_reconfigure")

(ros::roseus "test_dynamic_reconfigure_server" :anonymous nil)


(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("int_param"  int_t  1  "Int parameter" 0  -10  10)
         ("double_param"  double_t  2  "double parameter" 0.0  -2.0  10.0)
         ("str_param"  str_t  4  "String parameter" "foo")
         ("bool_param"  bool_t  8  "Boolean parameter"  nil))
        ;;; callbackfunc (defun func (config level) ) -> return updated-config
        '(lambda-closure nil 0 0 (cfg level)
           cfg)))


(init-unit-test)

(deftest test-dynamic-reconfigure-server-publish ()
  (let ((dsc-topic-name "/test_dynamic_reconfigure_server/parameter_descriptions")
        (upd-topic-name "/test_dynamic_reconfigure_server/parameter_updates")
        (dsc-msg nil)
        (upd-msg nil)
        (cnt 0))
    (ros::subscribe dsc-topic-name dynamic_reconfigure::ConfigDescription
                    #'(lambda (m) (setq dsc-msg m)))
    (ros::subscribe upd-topic-name dynamic_reconfigure::Config
                    #'(lambda (m) (setq upd-msg m)))

    (while (and (ros::ok) (< cnt 100) (or (null dsc-msg) (null upd-msg)))
      (incf cnt)
      (ros::spin-once)
      (ros::sleep))
    (ros::unsubscribe dsc-topic-name)
    (ros::unsubscribe upd-topic-name)

    (assert dsc-msg "~parameter_descriptions is not published.")
    (assert upd-msg "~parameter_updates is not published.")
    (let ((dsc-msg-max (send dsc-msg :max))
          (dsc-msg-min (send dsc-msg :min))
          (dsc-msg-dflt (send dsc-msg :dflt))
          (upd-msg-bool (car (send upd-msg :bools)))
          (upd-msg-int (car (send upd-msg :ints)))
          (upd-msg-str (car (send upd-msg :strs)))
          (upd-msg-double (car (send upd-msg :doubles))))

      ;; max
      (assert (equal (send (car (send dsc-msg-max :bools)) :name) "bool_param")
              "~parameter_descriptions/max/bools name is wrong")
      (assert (equal (send (car (send dsc-msg-max :bools)) :value) t)
              "~parameter_descriptions/max/bools value is wrong")
      (assert (equal (send (car (send dsc-msg-max :ints)) :name) "int_param")
              "~parameter_descriptions/max/ints name is wrong")
      (assert (equal (send (car (send dsc-msg-max :ints)) :value) 10)
              "~parameter_descriptions/max/ints value is wrong")
      (assert (equal (send (car (send dsc-msg-max :strs)) :name) "str_param")
              "~parameter_descriptions/max/strs name is wrong")
      (assert (equal (send (car (send dsc-msg-max :strs)) :value) "")
              "~parameter_descriptions/max/strs value is wrong")
      (assert (equal (send (car (send dsc-msg-max :doubles)) :name) "double_param")
              "~parameter_descriptions/max/doubles name is wrong")
      (assert (equal (send (car (send dsc-msg-max :doubles)) :value) 10.0)
              "~parameter_descriptions/max/doubles value is wrong")

      ;; min
      (assert (equal (send (car (send dsc-msg-min :bools)) :name) "bool_param")
              "~parameter_descriptions/min/bools name is wrong")
      (assert (equal (send (car (send dsc-msg-min :bools)) :value) nil)
              "~parameter_descriptions/min/bools value is wrong")
      (assert (equal (send (car (send dsc-msg-min :ints)) :name) "int_param")
              "~parameter_descriptions/min/ints name is wrong")
      (assert (equal (send (car (send dsc-msg-min :ints)) :value) -10)
              "~parameter_descriptions/min/ints value is wrong")
      (assert (equal (send (car (send dsc-msg-min :strs)) :name) "str_param")
              "~parameter_descriptions/min/strs name is wrong")
      (assert (equal (send (car (send dsc-msg-min :strs)) :value) "")
              "~parameter_descriptions/min/strs value is wrong")
      (assert (equal (send (car (send dsc-msg-min :doubles)) :name) "double_param")
              "~parameter_descriptions/min/doubles name is wrong")
      (assert (equal (send (car (send dsc-msg-min :doubles)) :value) -2.0)
              "~parameter_descriptions/min/doubles value is wrong")

      ;; dflt
      (assert (equal (send (car (send dsc-msg-dflt :bools)) :name) "bool_param")
              "~parameter_descriptions/dflt/bools name is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :bools)) :value) nil)
              "~parameter_descriptions/dflt/bools value is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :ints)) :name) "int_param")
              "~parameter_descriptions/dflt/ints name is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :ints)) :value) 0)
              "~parameter_descriptions/dflt/ints value is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :strs)) :name) "str_param")
              "~parameter_descriptions/dflt/strs name is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :strs)) :value) "foo")
              "~parameter_descriptions/dflt/strs value is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :doubles)) :name) "double_param")
              "~parameter_descriptions/dflt/doubles name is wrong")
      (assert (equal (send (car (send dsc-msg-dflt :doubles)) :value) 0.0)
              "~parameter_descriptions/dflt/doubles value is wrong")

      (assert (and upd-msg-bool (equal (send upd-msg-bool :name) "bool_param"))
              "~parameter_updates/bools name is wrong")
      (assert (and upd-msg-bool (equal (send upd-msg-bool :value) nil))
              "~parameter_updates/bools value is wrong")
      (assert (and upd-msg-int (equal (send upd-msg-int :name) "int_param"))
              "~parameter_updates/ints name is wrong")
      (assert (and upd-msg-int (equal (send upd-msg-int :value) 0))
              "~parameter_updates/ints value is wrong")
      (assert (and upd-msg-str (equal (send upd-msg-str :name) "str_param"))
              "~parameter_updates/strs name is wrong")
      (assert (and upd-msg-str (equal (send upd-msg-str :value) "foo"))
              "~parameter_updates/strs value is wrong")
      (assert (and upd-msg-double (equal (send upd-msg-double :name) "double_param"))
              "~parameter_updates/doubles name is wrong")
      (assert (and upd-msg-double (equal (send upd-msg-double :value) 0.0))
              "~parameter_updates/doubles value is wrong")
      )
    ))


(run-all-tests)

(exit)
