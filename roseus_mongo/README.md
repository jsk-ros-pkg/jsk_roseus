roseus_mongo
---

# parameters

### mongodb related parameters

- `*mongo-database*`: database name used by mongodb (default: param `robot/database` or "test")
- `*mongo-collection*`: collection name used by mongodb (default: param `robot/name` or "test")
- `*mongo-query-default-buffer-size*`: buffer size reserved for storing json raw string while translation (default: 4096 [byte])
- `*mongo-service-query*`: service name of querying message (default: "/message_store/query_messages")
- `*mongo-service-insert*`: service name of inserting message (default: "/message_store/insert")
- `*mongo-service-delete*`: service name of deleting message (default: "/message_store/delete")

### json(bson) related parameters

- `*json-parse-object-as*`: destination type of evaluating json object (`:alist` or `:plist`, default: `:alist`)
- `*json-parse-key-function*`: destination type of key when evaluation json object (`#'identity`, `#'string->keyword` or possible other functions, default: `#'string->keyword`)


# How to use

see [euslisp/mongo-client-sample.l](euslisp/mongo-client-sample.l)
