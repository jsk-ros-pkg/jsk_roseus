
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(ros::roseus "roseus_test_genmsg2")

(deftest test-msg-instance2
  (assert (ros::load-ros-manifest "roseus_test_genmsg2")
          "load-ros-manifest2")

  (assert (eval (read-from-string "(instance roseus_test_genmsg2::Child :init)"))
          "instantiating msg message2")

  )

(run-all-tests)

(exit)
