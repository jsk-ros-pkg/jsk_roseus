#!/usr/bin/env roseus
;;;
;;; ROS 2 version of test-geneus.l
;;; Only test-std-msgs is ported (pure serialize/deserialize test).
;;; test-send_msgs is skipped (requires Python ROS 2 publisher).
;;;
;;; Changes from ROS 1 version:
;;; - std_msgs::Duration, std_msgs::Time entries removed (not in ROS 2)
;;; - std_msgs::Header :seq entry removed (seq field removed in ROS 2)
;;; - std_msgs::Header :stamp uses builtin_interfaces::Time
;;; - roseus::FixedArray -> roseus::TestFixedArray
;;; - roseus::VariableArray -> roseus::TestVariableArray
;;; - time_data/duration_data use builtin_interfaces::Time/Duration instances

(require :unittest "lib/llib/unittest.l")

(ros::load-ros-manifest "roseus")

;;;
;;;
(init-unit-test)

(defun check-msg-gen (type &optional method data)
  (let (i o s)
    (if method
        (setq i (instance type :init method data))
      (setq i (instance type :init)))
    (setq o (instance type :init))
    (format *error-output* "~A ~A ~A " (send type :name) method data)
    (setq s (send i :serialize))
    (send o :deserialize s)
    (if method
        (progn
          (format *error-output* "~A~%" (send o method))
          (cond ((floatp data)
                 (assert (eps= (send o method) data)))
                ((float-vector-p data)
                 (assert (eps-v= (send o method) data)))
                (t
                 (assert (equal (send o method) data)))))
      (format *error-output* "~%"))
    ))

;; test codes
;; http://wiki.ros.org/msg#Fields
;; In ROS 2: byte is alias for uint8 (was int8 in ROS 1)
;; char: deprecated alias for uint8
(deftest test-std-msgs
  (mapcar #'(lambda (x) (apply #'check-msg-gen x))
          (list (list std_msgs::bool :data t)
                (list std_msgs::bool :data nil)
                (list std_msgs::byte :data 1)
                ;; byte :data -1 removed: byte is uint8 in ROS 2, -1 becomes 255
                (list std_msgs::bytemultiarray :data (make-array 3
                                                                  :element-type :char
                                                                  :initial-contents "eus"))
                (list std_msgs::char :data 2)
                (list std_msgs::colorrgba :r 0.1) (list std_msgs::colorrgba :g 0.2) (list std_msgs::colorrgba :b 0.3) (list std_msgs::colorrgba :a 0.4)
                ;; std_msgs::Duration and std_msgs::Time removed in ROS 2
                (list std_msgs::empty nil nil)
                (list std_msgs::float32 :data 1.234)
                (list std_msgs::float64 :data 5.678)
                ;; std_msgs::Header :seq removed in ROS 2
                (list std_msgs::header :stamp (instance builtin_interfaces::time :init :sec 1 :nanosec 234000000))
                (list std_msgs::header :frame_id "frame")
                (list std_msgs::int16 :data -1)
                (list std_msgs::int32 :data -2)
                (list std_msgs::int64 :data -3)
                (list std_msgs::int8 :data -4)
                (list std_msgs::string :data "euslisp")
                (list std_msgs::uint16 :data 1)
                (list std_msgs::uint32 :data 2)
                (list std_msgs::uint64 :data 3)
                (list std_msgs::uint8 :data 4)
                (list std_msgs::float32multiarray :data #f(1.234 5.678))
                (list std_msgs::float64multiarray :data #f(1.234 5.678))
                (list std_msgs::int16multiarray :data #i(1 2 3))
                (list std_msgs::int16multiarray :data #i(-1 -2 -3))
                (list std_msgs::int32multiarray :data #i(4 5 6))
                (list std_msgs::int32multiarray :data #i(-4 -5 -6))
                (list std_msgs::int64multiarray :data #i(7 8 9))
                (list std_msgs::int64multiarray :data #i(-7 -8 -9))
                (list std_msgs::int8multiarray :data #i(10 11 12))
                (list std_msgs::int8multiarray :data #i(-10 -11 -12))
                (list std_msgs::uint16multiarray :data #i(1 2 3))
                (list std_msgs::uint32multiarray :data #i(4 5 6))
                (list std_msgs::uint64multiarray :data #i(7 8 9))
                (list std_msgs::uint8multiarray :data (make-array 3
                                                                  :element-type :char
                                                                  :initial-contents "eus"))
                ;; TestFixedArray (ROS 2 version of FixedArray, using builtin_interfaces)
                (list roseus::TestFixedArray :float32_data #f(1 -2 3))
                (list roseus::TestFixedArray :float64_data #f(4 -5 6))
                (list roseus::TestFixedArray :int16_data #i(7 -8 9))
                (list roseus::TestFixedArray :int32_data #i(10 -11 12))
                (list roseus::TestFixedArray :int64_data #i(13 -14 15))
                (list roseus::TestFixedArray :int8_data #i(16 -17 18))
                (list roseus::TestFixedArray :uint16_data #i(1 2 3))
                (list roseus::TestFixedArray :uint32_data #i(4 5 6))
                (list roseus::TestFixedArray :uint64_data #i(7 8 9))
                (list roseus::TestFixedArray :uint8_data (make-array 17
                                                                     :element-type :char
                                                                     :initial-contents "johosystemkougaku"))
                (list roseus::TestFixedArray :bool_data (list t nil))
                (list roseus::TestFixedArray :time_data (list (instance builtin_interfaces::time :init :sec 0 :nanosec 0) (instance builtin_interfaces::time :init :sec 1 :nanosec 0)))
                (list roseus::TestFixedArray :duration_data (list (instance builtin_interfaces::duration :init :sec 0 :nanosec 0) (instance builtin_interfaces::duration :init :sec 1 :nanosec 0)))
                (list roseus::TestFixedArray :string_data (list (instance std_msgs::String :data "ros") (instance std_msgs::String :data "eus")))

                ;; TestVariableArray (ROS 2 version of VariableArray, using builtin_interfaces)
                (list roseus::TestVariableArray :float32_data #f(1 -2 3))
                (list roseus::TestVariableArray :float64_data #f(4 -5 6))
                (list roseus::TestVariableArray :int16_data #i(7 -8 9))
                (list roseus::TestVariableArray :int32_data #i(10 -11 12))
                (list roseus::TestVariableArray :int64_data #i(13 -14 15))
                (list roseus::TestVariableArray :int8_data #i(16 -17 18))
                (list roseus::TestVariableArray :uint16_data #i(1 2 3))
                (list roseus::TestVariableArray :uint32_data #i(4 5 6))
                (list roseus::TestVariableArray :uint64_data #i(7 8 9))
                (list roseus::TestVariableArray :uint8_data (make-array 17
                                                                     :element-type :char
                                                                     :initial-contents "johosystemkougaku"))
                (list roseus::TestVariableArray :bool_data (list t nil))
                (list roseus::TestVariableArray :time_data (list (instance builtin_interfaces::time :init :sec 0 :nanosec 0) (instance builtin_interfaces::time :init :sec 1 :nanosec 0)))
                (list roseus::TestVariableArray :duration_data (list (instance builtin_interfaces::duration :init :sec 0 :nanosec 0) (instance builtin_interfaces::duration :init :sec 1 :nanosec 0)))
                (list roseus::TestVariableArray :string_data (list (instance std_msgs::String :data "ros") (instance std_msgs::String :data "eus")))
                ))
  )

(ros::roseus "test_geneus")
(run-all-tests)
(exit)
