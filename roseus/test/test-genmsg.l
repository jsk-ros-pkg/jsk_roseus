
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(ros::roseus "roseus_test_genmsg")

(deftest test-msg-instance
  (assert (ros::load-ros-manifest "roseus_test_genmsg")
          "load-ros-manifest")

  (assert (eval (read-from-string "(instance roseus_test_genmsg::String :init)"))
          "instantiating msg message")
  )

(run-all-tests)

(exit)
