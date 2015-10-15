#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(ros::roseus "test_roseus_smach_utils")

(setq *sm* (make-state-machine
            '((:a -> :b)
              (:b -> :c)
              (:b !-> :b-fail)
              (:b-fail -> :b-recover)
              (:b-recover !-> :failed)
              (:b-recover -> :b)
              (:c -> :d)
              (:d -> :end))
            '((:a '(lambda (&optional args) (ros::ros-info "called :a") :success))
              (:b '(lambda (&optional args) (ros::ros-info "called :b") :fail))
              (:b-fail '(lambda (&optional args) (ros::ros-info "called :b-fail") :fail))
              (:b-recover '(lambda (&optional args) (ros::ros-info "called :b-recover") :fail))
              (:c '(lambda (&optional args) (ros::ros-info "called :c") :fail))
              (:d '(lambda (&optional args) (ros::ros-info "called :d") :success)))
            '(:a)
            '(:end :failed)
            :exec-result :success
            :exec-failure-result :fail))


(send *sm* :reset-state)
(setq *insp* (instance state-machine-inspector :init *sm*))
(send *insp* :publish-structure)

(init-unit-test)

(deftest test-methods
  (setq userdata nil)
  (assert (eq (send (send *sm* :start-state) :name) :a))
  (assert (equal (send-all (send *sm* :goal-state) :name) '(:end :failed)))
  (send *sm* :reset-state)
  (assert (not (send *sm* :goal-reached)))


  (setq node-a (send *sm* :active-state))
  (assert (find t (send-all (send node-a :arc-list) :check :success)))
  (assert (not (find t (send-all (send node-a :arc-list) :check t))))

  (setq node-b (send *sm* :node :b))
  (assert (null (set-exclusive-or '(t nil) (send-all (send node-b :arc-list) :check :success))))
  (assert (null (set-exclusive-or '(t nil) (send-all (send node-b :arc-list) :check :fail))))

  (send *sm* :active-state (send *sm* :node :b))
  (send *sm* :execute userdata :step 1)
  (assert (eq (send *sm* :active-state) (send *sm* :node :b-fail)))

  (send *sm* :active-state (send *sm* :node :d))
  (send *sm* :execute userdata :step 1)
  (assert (eq (send *sm* :active-state) (send *sm* :node :end))))


(run-all-tests)
(exit)
