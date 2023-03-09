#!/usr/bin/env roseus

;; It is important to load `TestName.l' before `String.l' to test for name clashing
;; between roseus::String and lisp::string
(flet ((directory (dir) (list "TestName.l" "StringStamped.l" "String.l")))
  (ros::roseus-add-files "roseus" "msg"))

(ros::roseus "test_name_sub")
(ros::subscribe "test_name" roseus::TestName #'(lambda (msg) (ros::ros-info "sub ~A" msg)))
(ros::spin)

