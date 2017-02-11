;; test-async-join-state-machine-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus_smach")
(ros::roseus "test_async_join_state_machine_actionlib")
(load "async-join-state-machine-sample.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(init-unit-test)

(deftest test-async-join-state-machine
  (setq *answer* nil)
  (setq *randint* nil)
  (setq *sm* (make-async-join-state-machine))
  (assert (eq (send (exec-state-machine *sm* '((random-int-value . 0))) :name) :success)
          "exec state machine with async join node")
  (assert (eq *answer* (+ (* *randint* 10) (- *randint* 5)))
          "check answer of computation"))

(run-all-tests)

(exit)

