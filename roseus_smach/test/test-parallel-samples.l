;; test-parallel-samples.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(load "package://roseus_smach/sample/state-machine-ros-sample.l")
(load "package://roseus_smach/sample/parallel-state-machine-sample.l")

(ros::roseus "test_parallel_samples")

(init-unit-test)

(make-sample-parallel-state-machine)

(deftest test-parallel-state-machine-sample
  (assert (eq (send (smach-exec *sm*) :name) :success)
          "parallel smach sample"))

(run-all-tests)
(exit)
