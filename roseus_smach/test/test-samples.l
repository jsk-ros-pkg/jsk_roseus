(require :unittest "lib/llib/unittest.l")

(load "package://roseus_smach/sample/state-machine-ros-sample.l")

(ros::roseus "test_roseus_smach_samples")

(init-unit-test)

(deftest test-smach-sample-simple ()
  (assert (eq (send (exec-smach-simple) :name) :outcome4)
	  "simple smach sample"))

(deftest test-smach-sample-nested ()
  (assert (eq (send (exec-smach-nested) :name) :outcome5)
	  "nested smach sample"))

(deftest test-smach-sample-userdata ()
  (assert (eq (send (exec-smach-userdata) :name) :outcome4)
	  "sample of smach with userdata")
  (assert (eq (send (exec-state-machine (smach-userdata)) :name) :outcome4)
          "exec (smach-userdata) without initial userdata"))

(deftest test-smach-action-client-state ()
  (setq userdata '(nil))
  (assert (eq (send (exec-state-machine (smach-action-client-state) userdata) :name) :SUCCEED-STATE)
          "exec (smach-action-server) is succeeded")
  (assert (cdr (assoc :result userdata))
          "action-client-state sets action result to userdata for key :result"))

(run-all-tests)

(exit)
