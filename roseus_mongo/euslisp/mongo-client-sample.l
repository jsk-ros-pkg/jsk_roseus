;; mongo-client-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

;; find all messages of type geometry_msgs/Pose
(mongo::query geometry_msgs::Pose)

;; find one message of all geometry_msgs/Pose
(mongo::query geometry_msgs::Pose :single t)

;; if you dont need meta data, use :msg-only
(mongo::query geometry_msgs::Pose :msg-only t)

;; find all geometry_msgs/Pose messages whose position.x = 2
(mongo::query geometry_msgs::Pose
              :query '((:position.x . 2)))

;; find all geometry_msgs/Pose sorted by dates newest to oldest
(mongo::query geometry_msgs::Pose
              :sort '((:$natural . -1)))

;; find latest 10 messages whose type is geometry_msgs/Pose
(mongo::query geometry_msgs::Pose
              :sort '((:$natural . -1))
              :limit 10)

;; find all geometry_msgs/Pose which were inserted after 2015/7/17 with ascending order
(mongo::query geometry_msgs::Pose
              :sort '((:$natural . 1))
              :meta '((:inserted_at .
                       ((:$gt . (send (instance calendar-time :init)
                                  :make :year 2015 :month 7 :day 17))))))

(setq msg (instance geometry_msgs::Pose :init))
;; insert message to database and wait for server to be inserted
(setq document-id (mongo::insert msg)) ;; => returns document id

;; you can also insert data without blocking
(mongo::insert msg :wait nil) ;; => returns t

;; you can also insert with annotating meta data
(mongo::insert msg :meta '((:memo . "my awesome pose")))

;; delete message from database
(setq success-p (mongo::delete-by-id document-id))

(provide :mongo-client-sample) ;; end of mongo-client-sample.l
