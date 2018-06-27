roseus_mongo
============

## parameters

### mongodb related parameters

- `mongo::*mongo-database*` (string, default: param `robot/database` or `test`)

  Database name used by mongodb

- `mongo::*mongo-collection*` (string, default: param `robot/name` or `test`)

  Collection name used by mongodb

- `mongo::*mongo-query-default-buffer-size*` (int, default 4096 [byte])

  Buffer size reserved for storing json raw string while serialization

- `mongo::*mongo-service-query*` (string, default: `/message_store/query_messages`)

  Service name for querying message
  
- `mongo::*mongo-service-insert*` (string, default: `/message_store/insert`)

  Service name for inserting message
  
- `mongo::*mongo-service-update*` (string, default: `/message_store/update`)

  Service name for updating message

- `mongo::*mongo-service-delete*` (string, default: `/message_store/delete`)

  Service name for deleting message
  
- `mongo::*mongo-service-timeout*` (int, default: param `~timeout` or 1)

  Seconds to wait for database server. Setting this value to `-1` means waiting forever.
  On euslisp only simulation without mongodb, it is recommended to set this value to `0` to avoid stuck.

### json(bson) related parameters

- `*json-parse-object-as*` (`:alist` or `:plist`, default: `:alist`)

  Destination type of evaluating json object
  
- `*json-parse-key-function*` (`#'identity`, `#'string->keyword` or possible other functions, default: `#'string->keyword`)

  Destination type of key when evaluation json object


## How to use

see [euslisp/mongo-client-sample.l](euslisp/mongo-client-sample.l)
