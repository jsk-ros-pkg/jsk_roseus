;; test-service-callback.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


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
  (ros::rate 1)
  (while (and (ros::ok)
              (< *callback-called-num* 2))
   (ros::ros-info "num: ~A" *callback-called-num*)
   (ros::spin-once)
   (ros::sleep))
  ;; this means callback is evaluated without segfault even when it does not return valid response.
  (assert t "callback is evaluated without segfault"))


(deftest test-call-unadvertised-service ()
  (ros::roseus "call_unadvertised_service")
  (call-empty-service "dummy"))


(run-all-tests)
(exit)


