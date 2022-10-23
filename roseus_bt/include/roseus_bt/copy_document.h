#ifndef BEHAVIOR_TREE_ROSEUS_BT_JSON_COPY_DOCUMENT_
#define BEHAVIOR_TREE_ROSEUS_BT_JSON_COPY_DOCUMENT_

#include "rapidjson/include/rapidjson/document.h"
#include "rapidjson/include/rapidjson/writer.h"
#include "rapidjson/include/rapidjson/prettywriter.h"
#include "rapidjson/include/rapidjson/stringbuffer.h"


namespace rapidjson
{
class CopyDocument : public rapidjson::Document
{
public:
  CopyDocument() : Document() {}

  CopyDocument(Type type) : Document(type) {}

  CopyDocument(const CopyDocument& document) {
    _clone(document);
  }

  CopyDocument(const Document& document) {
    _clone(document);
  }

  CopyDocument(CopyDocument&& document) : Document(std::move(document)) {}

  CopyDocument(Document&& document) : Document(std::move(document)) {}

  CopyDocument& operator=(const CopyDocument& document) {
    _clone(document);
    return *this;
  }

  CopyDocument& operator=(const Document& document) {
    _clone(document);
    return *this;
  }

  CopyDocument& operator=(CopyDocument&& document) {
    Swap(document);
    return *this;
  }

  CopyDocument& operator=(Document&& document) {
    Swap(document);
    return *this;
  }

  std::string toStr() const
  {
    StringBuffer strbuf;
    Writer<StringBuffer> writer(strbuf);
    Accept(writer);
    return strbuf.GetString();
  }

protected:
  void _clone(const Document& document) {
    SetObject();
    CopyFrom(document, GetAllocator());
  }
};

}  // namespace rapidjson

#endif  // BEHAVIOR_TREE_ROSEUS_BT_JSON_COPY_DOCUMENT_
