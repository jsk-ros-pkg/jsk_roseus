#!/usr/bin/env roseus
;;

(require :unittest "lib/llib/unittest.l")

(ros::roseus-add-msgs "actionlib_tutorials")
(ros::roseus "test-roseus")

;(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

(init-unit-test)

(setq *rate* 2)

(deftest test-rate ()
  (let (tm0 tm1 elapsed-time)
    (ros::rate *rate*)
    (while (= (send (ros::time-now) :to-sec) 0.0) (unix::sleep 1))) ;; wait for ros::time-now works, specially use_sim mode.
    (setq tm0 (ros::time-now))
    (setq tm1 (ros::time-now))
    (while (and (ros::ok) (< (send (ros::time- (ros::time-now) tm0) :to-sec) 10.0))
      (ros::sleep)
      (setq elapsed-time (send (ros::time- (ros::time-now) tm1) :to-sec))
      (ros::ros-info "measured rate ~A / duration ~A < 10.0" elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
      ;; skip first few seconds for use_sim mode
      (if (> (send (ros::time- (ros::time-now) tm0) :to-sec) 3.0)
          (assert (eps= elapsed-time (/ 1.0 *rate*) (* (/ 1.0 *rate*) 0.1)) "rate check"))
      (setq tm1 (ros::time-now))
      )
    ))

;; run after `rosrun actionlib_tutorials fibonacci_server.py`
(deftest test-rate-with-actionlib ()
  (let (tm0 elapsed-time c)
    (setq c (instance ros::simple-action-client :init
                      "fibonacci" actionlib_tutorials::FibonacciAction))
    (send c :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order 5)
    (send c :send-goal goal :feedback-cb #'(lambda (x) (ros::ros-info "feedback ... ~A" (send x :feedback :sequence))))
    (ros::rate *rate*)
    (setq tm0 (ros::time-now))
    (while (ros::ok)
      (ros::sleep)
      (setq elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
      (ros::ros-info "measured rate ~A" elapsed-time)
      (assert (eps= elapsed-time (/ 1.0 *rate*) (* (/ 1.0 *rate*) 0.1)) "rate check")
      (setq tm0 (ros::time-now))
      (when (> (length (send (send c :get-result) :sequence)) 0)
        (ros::ros-info "result ~A" (send (send c :get-result) :sequence))
        (return-from nil))
      (ros::spin-once) ;; to kick feedback-cb
      )
    ))

(deftest test-rate-with-actionlib-376-1 () ;; :send-goal and :wait-for-result within loop
  (let (tm0 elapsed-time c)
    (setq c (instance ros::simple-action-client :init
                      "fibonacci" actionlib_tutorials::FibonacciAction))
    (send c :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order 5)
    (ros::rate *rate*)
    (setq tm0 (ros::time-now))
    (while (ros::ok)
      (ros::ros-info "send goal")
      (send c :send-goal goal :feedback-cb #'(lambda (x) (ros::ros-info "feedback ... ~A" (send x :feedback :sequence))))
      (ros::ros-info "wait for result")
      (send c :wait-for-result)
      (ros::sleep)
      (setq elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
      (ros::ros-info "measured rate ~A" elapsed-time)
      (assert (eps>= elapsed-time (/ 1.0 *rate*) (* (/ 1.0 *rate*) 0.1)) "rate check")
      (setq tm0 (ros::time-now))
      (when (> (length (send (send c :get-result) :sequence)) 0)
        (ros::ros-info "result ~A" (send (send c :get-result) :sequence))
        (return-from nil))
      (ros::spin-once) ;; to kick feedback-cb
      )
    ))

(deftest test-rate-with-actionlib-376-2 () ;; :send-goal within loop
  (let ((i 0) tm0 elapsed-time c)
    (setq c (instance ros::simple-action-client :init
                      "fibonacci" actionlib_tutorials::FibonacciAction))
    (send c :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order 5)
    (ros::rate *rate*)
    (setq tm0 (ros::time-now))
    (while (ros::ok)
      (ros::ros-info "send goal")
      (send c :send-goal goal :feedback-cb #'(lambda (x) (ros::ros-info "feedback ... ~A" (send x :feedback :sequence))))
      (ros::ros-info "wait for result")
      (ros::sleep)
      (setq elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
      (ros::ros-info "measured rate ~A" elapsed-time)
      (assert (eps= elapsed-time (/ 1.0 *rate*) (* (/ 1.0 *rate*) 0.1)) "rate check")
      (setq tm0 (ros::time-now))
      (when (> (incf i) 5)
        (ros::ros-info "result ~A" (send (send c :get-result) :sequence))
        (return-from nil))
      (ros::spin-once) ;; to kick feedback-cb
      )
    ))

(deftest test-rate-with-actionlib-376-3 () ;; :send-goal within loop
  (let ((i 0) tm0 elapsed-time c)
    (setq c (instance ros::simple-action-client :init
                      "fibonacci" actionlib_tutorials::FibonacciAction))
    (send c :wait-for-server)
    (ros::rate *rate*)
    (setq tm0 (ros::time-now))
    (while (ros::ok)
      (ros::ros-info "send goal")
      (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
      (send goal :goal :order -1) ;; intentionally set wrong goal so that :wait-for-results returns immediately
      (send c :send-goal goal :feedback-cb #'(lambda (x) (ros::ros-info "feedback ... ~A" (send x :feedback :sequence))))
      (ros::ros-info "wait for result")
      (send c :wait-for-result)
      (ros::sleep)
      (setq elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
      (ros::ros-info "measured rate ~A" elapsed-time)
      (assert (eps= elapsed-time (/ 1.0 *rate*) (* (/ 1.0 *rate*) 0.1)) "rate check")
      (setq tm0 (ros::time-now))
      (when (> (incf i) 5)
        (ros::ros-info "result ~A" (send (send c :get-result) :sequence))
        (return-from nil))
      (ros::spin-once) ;; to kick feedback-cb
      )
    ))

(deftest test-rate-376 ()
  (let ((i 0) tm0 elapsed-time)
    (ros::rate *rate*)
    (setq tm0 (ros::time-now))
    (while (and (ros::ok) (<= (incf i) 10)) (ros::ros-info "run ~A" (send (ros::time- (ros::time-now) tm0) :to-sec)) (ros::sleep))
    (setq elapsed-time (send (ros::time- (ros::time-now) tm0) :to-sec))
    (ros::ros-info "measured rate ~A, expecting ~A" elapsed-time (* 10 (/ 1.0 *rate*)))
    (assert (eps= elapsed-time (* 10 (/ 1.0 *rate*)) (* (/ 1.0 *rate*) 0.1)) "rate check")
    ))

(run-all-tests)

(exit)
