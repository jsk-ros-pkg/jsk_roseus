
(require :unittest "lib/llib/unittest.l")

(load "package://roseus_smach/sample/state-machine-sample.l")
(load "package://roseus_smach/sample/state-machine-ros-sample.l")

(ros::roseus "test_roseus_smach_samples")

(init-unit-test)

(deftest test-sample-smach-simple
  (assert (eq (send (smach-exec-simple) :name) :outcome4)
	  "simple smach sample"))

(deftest test-sample-smach-nested
  (assert (eq (send (smach-exec-nested) :name) :outcome5)
	  "nested smach sample"))

(deftest test-sample-smach-userdata
  (assert (eq (send (smach-exec-userdata) :name) :outcome4)
	  "sample of smach with userdata"))

(run-all-tests)

(exit)
