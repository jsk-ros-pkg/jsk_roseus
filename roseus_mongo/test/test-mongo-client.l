;; test-mongo-client.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

(init-unit-test)

(deftest mongo-client-param ()
  (assert (string= mongo::*mongo-database* "test_database"))
  (assert (string= mongo::*mongo-collection* "test_collection")))

(deftest mongo-client ()

  (mongo::wait-for-server)

  (setq msg (instance geometry_msgs::Pose :init
                      :position (instance geometry_msgs::Point :init :x 1 :y 2 :z 3)))
  (setq doc-id (mongo::insert msg))
  (warn "doc-id: ~A~%" doc-id)

  (setq res (mongo::query geometry_msgs::Pose :query '((:_id . ((:$oid . doc-id)))) :single t :msg-only t))
  (print res)
  (assert (equal msg (car res)))

  (setq assume-nil (mongo::query geometry_msgs::TransformStamped :msg-only t))
  (assert (null assume-nil))

  (setq assume-fail (mongo::delete-by-id "11a1e111e111111e11000000"))
  (assert (not assume-fail))

  (setq assume-success (mongo::delete-by-id doc-id))
  (assert assume-success)
  (setq assume-deleted
        (mongo::query geometry_msgs::Pose :query '((:_id . ((:$oid . doc-id)))) :single t :msg-only t))
  (assert (null assume-deleted))

  (dotimes (i 10)
    (send msg :position :x i)
    (mongo::insert msg))

  (setq res-many (mongo::query geometry_msgs::Pose :sort '((:natural . 1 )) :msg-only t))
  (dotimes (i 10)
    (assert (eq (float i) (send (elt res-many i) :position :x))))

  ;; test non wait insertion
  ;; skips on hydro
  (unless (string= "hydro" (unix::getenv "ROS_DISTRO"))
    (mongo::insert (instance std_msgs::String :init :data "non wait") :wait nil)
    (unix:sleep 1)
    (assert (mongo::query std_msgs::String :query '((:data . "non wait")) :msg-only t)))

  ;; test update
  (setq msg-id (mongo::insert (instance std_msgs::UInt32 :init :data 2)))
  (assert (mongo::update-by-id msg-id (instance std_msgs::UInt32 :init :data 3)))
  (setq msg (mongo::query-by-id std_msgs::UInt32 msg-id :msg-only t))
  (assert (eq 3 (send msg :data)))

  ;; test if installed latest mongodb_store
  (setq amsg (instance mongodb_store_msgs::MongoQueryMsgRequest :init))
  (when (memq :limit (send amsg :methods))
    (setq assume-5-msgs (mongo::query geometry_msgs::Pose :sort '((:natural . 1)) :msg-only t :limit 5))
    (assert (= (length assume-5-msgs) 5))
    (dotimes (i 5)
      (assert (eq (float i) (send (elt assume-5-msgs i) :position :x)))))
)

(deftest mongo-client-timeout ()
  (setq mongo::*mongo-service-timeout* 1)
  (setq mongo::*mongo-service-insert* "/dummy/insert")

  (setq start-time (ros::time-now))
  (setq msg (instance geometry_msgs::Pose :init
                      :position (instance geometry_msgs::Point :init :x 1 :y 2 :z 3)))
  (setq doc-id (mongo::insert msg))
  (setq elapsed-time (ros::time- (ros::time-now) start-time))
  (assert (null doc-id))
  (assert (< (send elapsed-time :to-sec) 2)))


(run-all-tests)
(exit)

(provide :test-mongo-client) ;; end of test-mongo-client.l
