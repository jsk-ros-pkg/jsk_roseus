(load "unittest.l")

(load "package://roseus_smach/sample/state-machine-sample.l")
(load "package://roseus_smach/sample/state-machine-ros-sample.l")

(ros::roseus "test_roseus_smach_samples")

(init-unit-test)

(deftest test-smach-sample
  (assert (eq (send (smach-exec-simple) :name) :outcome4)
	  "simple smach sample")
  (assert (eq (send (smach-exec-nested) :name) :outcome5)
	  "nested smach sample")
  (assert (eq (send (smach-exec-userdata) :name) :outcome4)
	  "sample of smach with userdata")
  )

(run-all-tests)

(exit)
