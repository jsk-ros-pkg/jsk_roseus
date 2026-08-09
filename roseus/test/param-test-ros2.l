#!/usr/bin/env roseus
;;
;; ROS 2 version of param-test.l
;; In ROS 2, parameters are node-local (no global parameter server).
;;
;; Excluded tests:
;;   test-param-cache (get-param-cached unimplemented)
;;   test-list-param (list-param unimplemented)
;;   test-search-param (search-param unimplemented)
;;   test-delete-param (ROS 2 cannot undeclare statically typed params)
;;   test-dictionary-param (set-param stores dicts as string, get-param returns string)
;;   test-set-param-recursive (depends on dictionary support)
;;   global vs namespace vs private parameter distinction tests

(require :unittest "lib/llib/unittest.l")
(ros::roseus "read_parameter")

(init-unit-test)

(deftest test-set-param ()
  ;; In ROS 2, all parameters are node-local (no ~ prefix distinction)
  (let (ret)
    (ros::set-param "test_str" "test_value")
    (setq ret (ros::get-param "test_str"))
    (ros::ros-info "test_str ~A" ret)
    (assert (string= "test_value" ret) "[test_str] test_value != ~A" ret))

  (let (ret)
    (ros::set-param "test_int" 42)
    (setq ret (ros::get-param "test_int"))
    (ros::ros-info "test_int ~A" ret)
    (assert (= 42 ret) "[test_int] 42 != ~A" ret))

  (let (ret)
    (ros::set-param "test_float" 3.14)
    (setq ret (ros::get-param "test_float"))
    (ros::ros-info "test_float ~A" ret)
    (assert (eps= 3.14 ret 0.001) "[test_float] 3.14 != ~A" ret))

  (let (ret)
    (ros::set-param "test_bool_t" t)
    (setq ret (ros::get-param "test_bool_t"))
    (ros::ros-info "test_bool_t ~A" ret)
    (assert (eq t ret) "[test_bool_t] t != ~A" ret))

  (let (ret)
    (ros::set-param "test_bool_nil" nil)
    (setq ret (ros::get-param "test_bool_nil"))
    (ros::ros-info "test_bool_nil ~A" ret)
    (assert (null ret) "[test_bool_nil] nil != ~A" ret))
  )

(deftest test-get-param ()
  ;; Set params first, then get them
  (ros::set-param "get_test_str" "hello")
  (assert (ros::has-param "get_test_str") "[get_test_str] (ros::has-param) failed")
  (let ((ret (ros::get-param "get_test_str")))
    (ros::ros-info "get_test_str ~A" ret)
    (assert (string= "hello" ret) "[get_test_str] hello != ~A" ret))

  (ros::set-param "get_test_int" 12345)
  (assert (ros::has-param "get_test_int") "[get_test_int] (ros::has-param) failed")
  (let ((ret (ros::get-param "get_test_int")))
    (ros::ros-info "get_test_int ~A" ret)
    (assert (= 12345 ret) "[get_test_int] 12345 != ~A" ret))
  )

(deftest test-has-param ()
  (ros::set-param "has_param_test" "exists")
  (assert (ros::has-param "has_param_test") "(ros::has-param has_param_test) should return t")
  (assert (not (ros::has-param "param_definitely_not_exists")) "(ros::has-param param_definitely_not_exists) should return nil")
  )

(deftest test-param-default-value ()
  ;; Read parameter with default value (parameter does not exist)
  (dotimes (i 100)
    (assert (equal "foo" (ros::get-param "nonexistent_bar_param" "foo"))
            "(ros::get-param nonexistent_bar_param foo) should return foo"))
  )

(deftest test-param-overwrite ()
  ;; Test overwriting a parameter
  (ros::set-param "overwrite_test" "initial")
  (assert (string= (ros::get-param "overwrite_test") "initial"))
  (ros::set-param "overwrite_test" "updated")
  (assert (string= (ros::get-param "overwrite_test") "updated"))
  )

(run-all-tests)

(exit)
