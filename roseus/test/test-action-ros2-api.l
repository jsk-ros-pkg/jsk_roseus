#!/usr/bin/env roseus
;;;
;;; ROS 2-style action API tests
;;; Tests: feedback callback, cancel goal, send-goal-and-wait, get-state
;;; Requires fibonacci-server-ros2-async.l (supports cooperative cancellation)
;;;
(ros::load-ros-manifest "roseus")
(require :actionlib2 "actionlib2.l")

(ros::roseus "test_action_ros2_api")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; Single client reused across all tests
(setq *c* (instance ros::simple-action-client :init
                    "fibonacci" roseus::Fibonacci))
(warning-message 2 ";; wait-for-server~%")
(send *c* :wait-for-server)

;;;
;;; Test 1: send-goal-and-wait
;;;
(deftest test-send-goal-and-wait
  (let (result r)
    ;; send-goal-and-wait with a Goal object
    (let ((goal (instance roseus::Fibonacci_Goal :init)))
      (send goal :order 5)
      (warning-message 2 ";; send-goal-and-wait order=5~%")
      (assert (send *c* :send-goal-and-wait goal :timeout 30)
              "send-goal-and-wait should return t")
      (setq r (send *c* :get-result))
      (assert r "get-result should not be nil")
      (when r
        (setq result (send r :sequence))
        (warning-message 2 ";; result -> ~A~%" result)
        ;; fib(5) = [0, 1, 1, 2, 3, 5]
        (assert (= (length result) 6) "length should be 6, got ~A" (length result))
        (assert (= (elt result 5) 5) "last element should be 5, got ~A" (elt result 5))))

    ;; send-goal-and-wait with a SendGoalRequest (ROS 1 compat)
    (let ((goal (instance roseus::Fibonacci_SendGoalRequest :init)))
      (send goal :goal :order 8)
      (warning-message 2 ";; send-goal-and-wait order=8 (SendGoalRequest)~%")
      (assert (send *c* :send-goal-and-wait goal :timeout 30)
              "send-goal-and-wait with SendGoalRequest should return t")
      (setq r (send *c* :get-result))
      (assert r "get-result should not be nil for order=8")
      (when r
        (setq result (send r :sequence))
        (warning-message 2 ";; result -> ~A~%" result)
        ;; fib(8) = [0, 1, 1, 2, 3, 5, 8, 13, 21]
        (assert (= (length result) 9) "length should be 9, got ~A" (length result))
        (assert (= (elt result 8) 21) "last element should be 21, got ~A" (elt result 8))))))

;;;
;;; Test 2: get-state transitions
;;;
(deftest test-get-state
  (let (goal state)
    (warning-message 2 ";; test-get-state~%")

    ;; Send a goal and wait for result
    (setq goal (instance roseus::Fibonacci_Goal :init))
    (send goal :order 5)
    (send *c* :send-goal goal)
    (warning-message 2 ";; goal sent, state = ~A~%"
                     (send *c* :get-goal-status-string))

    ;; Wait for result with timeout
    (assert (send *c* :wait-for-result :timeout 30)
            "wait-for-result should return t")
    (setq state (send *c* :get-state))
    (warning-message 2 ";; after result, state = ~A (~A)~%"
                     state (send *c* :get-goal-status-string))
    ;; Final state should be SUCCEEDED
    (assert (= state ros::*action-goal-state-succeeded*)
            "final state should be SUCCEEDED(4), got ~A (~A)"
            state (send *c* :get-goal-status-string))))

;;;
;;; Test 3: Feedback callback
;;;
(deftest test-feedback-callback
  (let (goal feedback-count feedback-list r)
    (warning-message 2 ";; test-feedback-callback~%")

    ;; Track feedback via callback
    (setq feedback-count 0)
    (setq feedback-list nil)
    (setq goal (instance roseus::Fibonacci_Goal :init))
    (send goal :order 8)
    (send *c* :send-goal goal
          :feedback-cb #'(lambda (fb)
                           (incf feedback-count)
                           (push (send fb :sequence) feedback-list)))
    (assert (send *c* :wait-for-result :timeout 30)
            "wait-for-result with feedback should return t")

    (warning-message 2 ";; feedback-count = ~A~%" feedback-count)
    (assert (> feedback-count 0)
            "should have received at least 1 feedback, got ~A" feedback-count)
    ;; Each feedback should be a growing sequence
    (setq feedback-list (nreverse feedback-list))
    (when (> (length feedback-list) 1)
      (assert (> (length (car (last feedback-list)))
                 (length (car feedback-list)))
              "later feedback should have longer sequence"))
    (warning-message 2 ";; feedback sequences: ~A~%" feedback-list)

    ;; Result should still be correct
    (setq r (send *c* :get-result))
    (assert r "get-result should not be nil after feedback test")
    (when r
      (let ((result (send r :sequence)))
        (assert (= (elt result (1- (length result))) 21)
                "fib(8) last should be 21, got ~A" (elt result (1- (length result))))))))

;;;
;;; Test 4: Cancel goal
;;;
(deftest test-cancel-goal
  (let (goal state r)
    (warning-message 2 ";; test-cancel-goal~%")

    ;; Send a large goal (order=50) that takes a while to compute
    (setq goal (instance roseus::Fibonacci_Goal :init))
    (send goal :order 50)
    (send *c* :send-goal goal)
    (warning-message 2 ";; goal sent, spinning to let server start...~%")

    ;; Give server time to start processing
    (dotimes (i 30) (send *c* :spin-once) (unix::usleep 10000))
    (warning-message 2 ";; state before cancel = ~A (~A)~%"
                     (send *c* :get-state) (send *c* :get-goal-status-string))

    ;; Cancel the goal
    (warning-message 2 ";; canceling goal...~%")
    (send *c* :cancel-goal)

    ;; Spin to process cancel response and let server process cancel
    (warning-message 2 ";; spinning after cancel...~%")
    (dotimes (i 30) (send *c* :spin-once) (unix::usleep 10000))
    (warning-message 2 ";; state after cancel spin = ~A (~A)~%"
                     (send *c* :get-state) (send *c* :get-goal-status-string))

    ;; Wait for result (should come back as canceled)
    (warning-message 2 ";; waiting for result after cancel~%")
    (let ((got-result (send *c* :wait-for-result :timeout 10)))
      (warning-message 2 ";; wait-for-result returned ~A~%" got-result)
      (assert got-result "should receive result after cancel")

      (setq state (send *c* :get-state))
      (warning-message 2 ";; final state = ~A (~A)~%"
                       state (send *c* :get-goal-status-string))
      ;; State should be CANCELED
      (assert (= state ros::*action-goal-state-canceled*)
              "state after cancel should be CANCELED(5), got ~A (~A)"
              state (send *c* :get-goal-status-string))

      ;; Result should contain partial sequence
      (setq r (send *c* :get-result))
      (warning-message 2 ";; get-result = ~A~%" r)
      (assert r "get-result should not be nil after cancel")
      (when r
        (let ((result (send r :sequence)))
          (warning-message 2 ";; partial result length = ~A~%" (length result))
          (assert (< (length result) 51)
                  "canceled result should be shorter than full sequence"))))))

;;;
;;; Test 5: send-goal-and-wait with timeout
;;;
(deftest test-send-goal-and-wait-timeout
  (let (goal ret)
    (warning-message 2 ";; test-send-goal-and-wait-timeout~%")

    ;; Send a large goal with short timeout
    (setq goal (instance roseus::Fibonacci_Goal :init))
    (send goal :order 200)
    (warning-message 2 ";; send-goal-and-wait with 2s timeout, order=200~%")
    (setq ret (send *c* :send-goal-and-wait goal :timeout 2))
    (warning-message 2 ";; send-goal-and-wait returned ~A~%" ret)
    ;; Should timeout (return nil)
    (assert (null ret)
            "send-goal-and-wait with short timeout should return nil")

    ;; Cancel the goal so server is ready for next tests
    (send *c* :cancel-goal)
    ;; Wait for cancel to take effect
    (dotimes (i 50) (send *c* :spin-once) (unix::usleep 10000))))

(run-all-tests)
(exit)
