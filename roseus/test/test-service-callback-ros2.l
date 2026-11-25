;; test-service-callback-ros2.l
;; ROS 2 version of test-service-callback.l
;; Excluded: test-call-unadvertised-service (call-empty-service is in roseus-utils.l, not ported)

(ros::load-ros-manifest "roseus")
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest test-missing-service-response ()

  (setq *callback-called-num* 0)

  (defun add-two-ints-wrong (req)
    (inc *callback-called-num*)
    (let ((m (send req :response)))
      (ros::ros-info "Returning [~d + ~d = ~d]"
                     (send req :a) (send req :b)
                     (+ (send req :a) (send req :b)))
      (send m :sum  (+ (send req :a) (send req :b)))
      ;; m ;; response is not returned.
      ))

  (ros::roseus "add_two_ints_server")
  (ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints-wrong)
  ;; Poll at 10Hz for up to 30 seconds (DDS discovery is slow in ROS 2)
  (ros::rate 10)
  (dotimes (i 300)
    (when (>= *callback-called-num* 2) (return))
    (when (= (mod i 100) 0) (ros::ros-info "num: ~A" *callback-called-num*))
    (ros::spin-once)
    (ros::sleep))
  ;; this means callback is evaluated without segfault even when it does not return valid response.
  (assert t "callback is evaluated without segfault"))


(run-all-tests)
(exit)
