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
              (:b-fail !-> :failed)
              (:b-recover !-> :failed)
              (:b-recover -> :b)
              (:c -> :d)
              (:d -> :end))
            '((:a '(lambda (userdata) (ros::ros-info "called :a") :success))
              (:b
                '(lambda (userdata)
                   (assert (eq (cdr (assoc 'count userdata)) 0))
                   (incf (cdr (assoc 'count userdata)))
                   (assert (eq (cdr (assoc 'count userdata)) 1))
                   (ros::ros-info "userdata: ~A" userdata)
                   (ros::ros-info "called :b")
                   :fail))
              (:b-fail
                '(lambda (userdata)
                   (incf (cdr (assoc 'count userdata)))
                   (assert (eq (cdr (assoc 'count userdata)) 2))
                   (ros::ros-info "userdata: ~A" userdata)
                   (ros::ros-info "called :b-fail")
                   :fail))
              (:b-recover '(lambda (userdata) (ros::ros-info "called :b-recover") :fail))
              (:c '(lambda (userdata) (ros::ros-info "called :c") :fail))
              (:d '(lambda (userdata) (ros::ros-info "called :d") :success)))
            '(:a)
            '(:end :failed)
            :exec-result :success
            :exec-failure-result :fail))


(send *sm* :reset-state)
(setq *insp* (instance state-machine-inspector :init *sm*))
(send *insp* :publish-structure)

(init-unit-test)

(deftest test-methods
  (setq userdata '((count . 0)))
  (assert (eq (send (send *sm* :start-state) :name) :a))
  (assert (equal (send-all (send *sm* :goal-state) :name) '(:end :failed)))
  (send *sm* :reset-state)
  (send *sm* :arg-keys 'count)
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

(deftest test-exec-state-machine
  (exec-state-machine *sm* '((count . 0)) :spin t :hz 0.5 :root-name "SM_ROOT"))

(run-all-tests)
(exit)
