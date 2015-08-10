;; mongo-client.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "roseus_mongo_client")

(ros::load-ros-manifest "mongodb_store_msgs")

(require :json-encode "json/json-encode.l")
(require :json-decode "json/json-decode.l")

(defvar *mongo-database* (ros::get-param "mongodb/database" "test"))
(defvar *mongo-collection* (ros::get-param "robot/name" "test"))
(defvar *mongo-query-default-buffer-size* 4096)
(defvar *mongo-service-query* "/message_store/query_messages")
(defvar *mongo-service-insert* "/message_store/insert")
(defvar *mongo-service-delete* "/message_store/delete")

(unless (find-package "MONGO") (make-package "MONGO"))
(in-package "MONGO")

(defun wait-for-server ()
  (ros::wait-for-service "/datacentre/wait_ready")
  (user::call-empty-service "/datacentre/wait_ready")
  (unix:sleep 2))

(defun shutdown-server ()
  (ros::wait-for-service "/datacentre/shutdown")
  (user::call-empty-service "/datacentre/shutdown"))

(defun apply-to-ros-query (encoder lst)
  (let ((ss (make-string-output-stream user::*mongo-query-default-buffer-size*)))
    (funcall encoder lst ss)
    (ros::ros-debug "query: ~A~%" (get-output-stream-string ss))
    (instance mongodb_store_msgs::StringPairList :init
              :pairs
              (list
               (instance mongodb_store_msgs::StringPair :init
                         :first mongodb_store_msgs::MongoQueryMsgRequest::*JSON_QUERY*
                         :second (get-output-stream-string ss))))))

(defun alist->ros-query (alist)
  (apply-to-ros-query #'json::encode-alist alist))

(defun plist->ros-query (plist)
  (apply-to-ros-query #'json::encode-plist plist))

(defun query (msg-type &key single query meta sort (encoder #'json::encode-alist) msg-only)
  (let ((req (instance mongodb_store_msgs::MongoQueryMsgRequest :init))
        res)
    (send req :database user::*mongo-database*)
    (send req :collection user::*mongo-collection*)
    (send req :type (get msg-type :datatype-))
    (send req :single single)
    (when query
      (send req :message_query (apply-to-ros-query encoder query)))
    (when meta
      (send req :meta_query (apply-to-ros-query encoder meta)))
    (when sort
      (send req :sort_query (apply-to-ros-query encoder sort)))
    (ros::wait-for-service user::*mongo-service-query*)
    (setq res (ros::service-call user::*mongo-service-query* req))
    (mapcar #'(lambda (message meta-pair)
                (let ((msg (instance msg-type :init))
                      (meta-raw-string (send (car (send meta-pair :pairs)) :second)))
                  (send msg :deserialize (send message :msg))
                  (if msg-only msg
                      (cons msg (json::parse-from-string meta-raw-string)))))
            (send res :messages) (send res :metas))))

(defun insert (msg &key meta (encoder #'json::encode-alist))
  (let ((req (instance mongodb_store_msgs::MongoInsertMsgRequest :init))
        res)
    (send req :database user::*mongo-database*)
    (send req :collection user::*mongo-collection*)
    (send req :message (instance mongodb_store_msgs::SerialisedMessage :init
                                 :type (send msg :datatype-)
                                 :msg (send msg :serialize)))
    (when meta
      (send req :meta (apply-to-ros-query encoder meta)))
    (ros::wait-for-service user::*mongo-service-insert*)
    (setq res (ros::service-call user::*mongo-service-insert* req))
    (send res :id)))

(defun delete-by-id (id)
  (let ((req (instance mongodb_store_msgs::MongoDeleteMsgRequest :init))
        res)
    (send req :database user::*mongo-database*)
    (send req :collection user::*mongo-collection*)
    (send req :document_id id)
    (ros::wait-for-service user::*mongo-service-delete*)
    (setq res (ros::service-call user::*mongo-service-delete* req))
    (send res :success)))

(defun replicate (&key
                    (database *mongo-database*)
                    (colections (list *mongo-collection*))
                    (move-before (ros::time (* 60 60 24)))
                    (delete-after-move nil))
  (let ((c (instance ros::simple-action-client :init
                     "move_mongodb_entries" mongodb_store_msgs::MoveEntriesAction))
        (goal (instance mongodb_store_msgs::MoveEntriesActionGoal :init
                        :database database
                        :collections (instance mongodb_store_msgs::StringList :init
                                               :data collections)
                        :move_before move-before
                        :delete_after_move delete-after-move)))
    (send c :send-goel goal)
    (send c :wait-for-result)
    (send (send c :get-result) :completed)))

(provide :mongo-client) ;; end of mongo-client.l
