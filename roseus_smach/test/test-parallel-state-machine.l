;; test-parallel-state-machine.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/sample/parallel-state-machine-sample.l")

(ros::roseus "test_parallel_state_machine")

(init-unit-test)

(make-sample-parallel-state-machine)

(deftest test-parallel-state-machine
  (assert (eq (send (exec-state-machine *sm*) :name) :success)
          "test parallel state machine sample"))

(deftest test-parallel-state-machine-with-root-name
  (assert (eq (send (exec-state-machine *sm* nil :root-name "SM_ROOT_1") :name) :success)
          "test parallel state machine sample with root name"))

(run-all-tests)

(exit)

