(require :unittest "lib/llib/unittest.l")

(load "package://roseus_smach/sample/state-machine-ros-sample.l")
(load "package://roseus_smach/sample/parallel-state-machine-sample.l")

(ros::roseus "test_roseus_smach_graph_samples")

(init-unit-test)

(deftest smach-graph-goal ()
  (let (s)
    (setq s (smach-simple2))
    (warning-message 2 "write to /tmp/smach.pdf~%")
    (send s :write-to-pdf "/tmp/smach.pdf" nil "smach")
    ;; solve as graph search
    (setq *solver* (instance breadth-first-graph-search-solver))

    ;; setting start/goal state by node state
    (send s :start-state (send s :node :bar))
    (send s :goal-state (send s :node :outcome4))
    (setq path (send *solver* :solve s :verbose t))
    (warning-message 4 "solved path ~A~%" (send-all (send-all path :state) :name))
    (assert (equal (send-all (send-all path :state) :name)
                   '(:bar :foo :outcome4)))

    ;; setting start/goal state by symbol
    (send s :start-state :bar)
    (send s :goal-state :outcome4)
    ;;;
    (send *solver* :clear-open-list) ;;; need to clear open list before you re-use *solver*
    ;;;
    (setq path (send *solver* :solve s :verbose t))
    (warning-message 4 "solved path ~A~%" (send-all (send-all path :state) :name))
    (assert (equal (send-all (send-all path :state) :name)
                   '(:bar :foo :outcome4)))

    ))

(run-all-tests)

(exit)
