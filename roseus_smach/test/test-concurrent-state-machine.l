;; test-parallel-state-machine.l
;; Author: Hitoshi Kamada <h-kamada@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/sample/concurrent-state-machine-sample.l")

(ros::roseus "test_concurrent_state_machine")

(init-unit-test)

(make-sample-concurrent-state-machine)

(deftest test-concurrent-state-machine
  (assert (eq (send (smach-exec *sm*) :name) :success)
          "test concurrent state machine sample"))

(run-all-tests)

(exit)
