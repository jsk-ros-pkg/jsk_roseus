/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// author: Kei Okada


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>

#include <cstdio>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include <ros/init.h>
#include <ros/time.h>
#include <ros/rate.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/service_server_link.h>
#include <ros/service_manager.h>
#include <ros/connection.h>
#include <rospack/rospack.h>
#include <ros/param.h>
#include <ros/callback_queue.h>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string

#include "eus.h"
extern "C" {
#ifdef ROSPACK_EXPORT
  rospack::ROSPack rp;
#else
  rospack::Rospack rp;
#endif
  pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env);
  void register_roseus(){
    char modname[] = "___roseus";
    return add_module_initializer(modname, (pointer (*)())___roseus);}
  /* get_string is originally defined in lisp/c/makes.c, but it is also defined in Python.so linked from rospack.so
     to avoid confusion, we have to explictly re-define this method, specially for OSX */
  byte *get_string(register pointer s){
    if (isstring(s)) return(s->c.str.chars);
    if (issymbol(s)) return(s->c.sym.pname->c.str.chars);
    else error(E_NOSTRING); return NULL; }
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

namespace ros {
  namespace master {
    std::string g_uri;
    void init(const M_string& remappings);
  }
  namespace param {
    void init(const M_string& remappings);
  }
}

using namespace ros;
using namespace std;

#define isInstalledCheck \
  if( ! ros::ok() ) { error(E_USER,"You must call (ros::roseus \"name\") before creating the first NodeHandle"); }

class RoseusStaticData
{
public:
  RoseusStaticData() {}
  ~RoseusStaticData() {
  }
  boost::shared_ptr<ros::NodeHandle> node;
  boost::shared_ptr<ros::Rate> rate;
  map<string, boost::shared_ptr<Publisher> > mapAdvertised; ///< advertised topics
  map<string, boost::shared_ptr<Subscriber> > mapSubscribed; ///< subscribed topics
  map<string, boost::shared_ptr<ServiceServer> > mapServiced; ///< subscribed topics
  map<string, Timer > mapTimered; ///< subscribed timers

  map<string, boost::shared_ptr<NodeHandle> > mapHandle; ///< for grouping nodehandle
};

static RoseusStaticData s_staticdata;
static bool s_bInstalled = false;
#define s_node s_staticdata.node
#define s_rate s_staticdata.rate
#define s_mapAdvertised s_staticdata.mapAdvertised
#define s_mapSubscribed s_staticdata.mapSubscribed
#define s_mapServiced s_staticdata.mapServiced
#define s_mapTimered s_staticdata.mapTimered
#define s_mapHandle s_staticdata.mapHandle

pointer K_ROSEUS_MD5SUM,K_ROSEUS_DATATYPE,K_ROSEUS_DEFINITION,K_ROSEUS_CONNECTION_HEADER,K_ROSEUS_SERIALIZATION_LENGTH,K_ROSEUS_SERIALIZE,K_ROSEUS_DESERIALIZE,K_ROSEUS_INIT,K_ROSEUS_GET,K_ROSEUS_REQUEST,K_ROSEUS_RESPONSE,K_ROSEUS_GROUPNAME,K_ROSEUS_ONESHOT,K_ROSEUS_LAST_EXPECTED,K_ROSEUS_LAST_REAL,K_ROSEUS_CURRENT_EXPECTED,K_ROSEUS_CURRENT_REAL,K_ROSEUS_LAST_DURATION,K_ROSEUS_SEC,K_ROSEUS_NSEC,QANON,QNOOUT,QREPOVERSION,QROSDEBUG,QROSINFO,QROSWARN,QROSERROR,QROSFATAL;
extern pointer LAMCLOSURE;

/***********************************************************
 *   Message wrapper
 ************************************************************/

string getString(pointer message, pointer method) {
  context *ctx = current_ctx;
  pointer r, curclass;
  if ((pointer)findmethod(ctx,method,classof(message),&curclass)!=NIL) {
    r = csend(ctx,message,method,0);
  } else if ((pointer)findmethod(ctx,K_ROSEUS_GET,classof(message),&curclass) != NIL ) {
    r = csend(ctx,message,K_ROSEUS_GET,1,method);
  } else {
    r = NULL;
#ifdef x86_64
    ROS_ERROR("could not find method %s for pointer %lx",
              get_string(method), (long unsigned int)message);
#else
    ROS_ERROR("could not find method %s for pointer %x",
              get_string(method), (unsigned int)message);
#endif
  }
  if ( !isstring(r) ) {
    pointer dest=(pointer)mkstream(ctx,K_OUT,makebuffer(64));
    prinx(ctx,message,dest);
    pointer str = makestring((char *)dest->c.stream.buffer->c.str.chars,
                             intval(dest->c.stream.count));
    ROS_ERROR("send %s to %s returns nil", get_string(method), get_string(str));
  }
  ROS_ASSERT(isstring(r));
  string ret = (char *)get_string(r);
  return ret;
}

int getInteger(pointer message, pointer method) {
  context *ctx = current_ctx;
  pointer a,curclass;
  vpush(message);
  a = (pointer)findmethod(ctx,method,classof(message),&curclass);
  if (a!=NIL) {
    pointer r = csend(ctx,message,method,0);
    vpop();                     // message
    return (ckintval(r));
  } else {
#ifdef x86_64
    ROS_ERROR("could not find method %s for pointer %lx",
              get_string(method), (long unsigned int)message);
#else
    ROS_ERROR("could not find method %s for pointer %x",
              get_string(method), (unsigned int)message);
#endif
    vpop();                     // message
  }
  return 0;
}

class EuslispMessage
{
public:
  pointer _message;
  boost::shared_ptr<map<string, string> > _connection_header;

  EuslispMessage(pointer message) : _message(message) {
  }
  EuslispMessage(const EuslispMessage &r) {
    context *ctx = current_ctx;
    if (ctx!=euscontexts[0])ROS_WARN("ctx is not correct %d\n",thr_self());
    if ( isclass(r._message) ) {
      //ROS_ASSERT(isclass(r._message));
      vpush(r._message);
      _message = makeobject(r._message);
      vpush(_message);
      csend(ctx,_message,K_ROSEUS_INIT,0);
      vpop();                   // _message
      vpop();                   // r._message
    } else {
      ROS_WARN("r._message must be class");prinx(ctx,r._message,ERROUT);flushstream(ERROUT);terpri(ERROUT);
      _message = r._message;
    }
  }
  virtual ~EuslispMessage() { }

  virtual void replaceContents (pointer newMessage) {
    _message = newMessage;
  }

  virtual const string __getDataType() const {
    return getString(_message, K_ROSEUS_DATATYPE);
  }
  virtual const string __getMD5Sum()   const {
    return getString(_message, K_ROSEUS_MD5SUM);
  }
  virtual const string __getMessageDefinition() const {
    return getString(_message, K_ROSEUS_DEFINITION);
  }
  virtual const string __getServiceDataType() const {
    return getString(_message, K_ROSEUS_DATATYPE);
  }
  virtual const string __getServerMD5Sum()    const {
    return getString(_message, K_ROSEUS_MD5SUM);
  }

  uint32_t serializationLength() const {
    context *ctx = current_ctx;
    if (ctx!=euscontexts[0])ROS_WARN("ctx is not correct %d\n",thr_self());
    pointer a,curclass;
    a = (pointer)findmethod(ctx,K_ROSEUS_SERIALIZATION_LENGTH,classof(_message),&curclass);
    ROS_ASSERT(a!=NIL);
    return getInteger(_message, K_ROSEUS_SERIALIZATION_LENGTH);
  }

  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
  {
    context *ctx = current_ctx;
    if (ctx!=euscontexts[0])ROS_WARN("ctx is not correct %d\n",thr_self());
    pointer a,curclass;
    vpush(_message);            // to avoid GC
    uint32_t len = serializationLength();
    vpop();                     // pop _message
    a = (pointer)findmethod(ctx,K_ROSEUS_SERIALIZE,classof(_message),&curclass);
    ROS_ASSERT(a!=NIL);
    pointer r = csend(ctx,_message,K_ROSEUS_SERIALIZE,0);
    ROS_ASSERT(isstring(r));
    memcpy(writePtr, get_string(r), len);
    //ROS_INFO("serialize");

    return writePtr + len;
  }

  virtual uint8_t *deserialize(uint8_t *readPtr, uint32_t sz)
  {
    context *ctx = current_ctx;
    if (ctx!=euscontexts[0])ROS_WARN("ctx is not correct %d\n",thr_self());
    pointer a,curclass;

    if ( sz == 0 ) {
      ROS_DEBUG("empty message!");
      return readPtr;
    }
    vpush(_message);
    a = (pointer)findmethod(ctx,K_ROSEUS_DESERIALIZE,classof(_message),&curclass);
    ROS_ASSERT(a!=NIL);
    pointer p = makestring((char *)readPtr, sz);
    pointer r;
    r = csend(ctx,_message,K_ROSEUS_DESERIALIZE,1,p);
    ROS_ASSERT(r!=NIL);
    //ROS_INFO("deserialize %d", __serialized_length);
    vpop(); // pop _message

    return readPtr + sz;
  }
};

void StoreConnectionHeader(EuslispMessage *eus_msg) {
  if ( eus_msg->_connection_header == NULL ||
       eus_msg->_connection_header->size() == 0 ) {
    return;
  }
  context *ctx = current_ctx;
  // store conection headers
  register pointer ret, header;
  ret = cons(ctx, NIL, NIL);
  header = ret;
  vpush(ret);
  for(map<string, string>::iterator it = eus_msg->_connection_header->begin(); it != eus_msg->_connection_header->end(); it++){
    ccdr(ret) = cons(ctx,cons(ctx,makestring((char *)it->first.c_str(), it->first.length()),
                              makestring((char *)it->second.c_str(), it->second.length())),NIL);
    ret = ccdr(ret);
  }
  /* (setslot obj class index newval) */
  pointer slot_args[4] = {eus_msg->_message, classof(eus_msg->_message), K_ROSEUS_CONNECTION_HEADER, ccdr(header)};
  SETSLOT(ctx, 4, slot_args);
  vpop();
}

namespace ros{
  namespace serialization{
template<> struct Serializer<EuslispMessage> {
  template<typename Stream>
  inline static void write(Stream& stream, boost::call_traits<EuslispMessage>::param_type t) {
    t.serialize(stream.getData(), 0);
  }

  template<typename Stream>
  inline static void read(Stream& stream, boost::call_traits<EuslispMessage>::reference t) {
    t.deserialize(stream.getData(), stream.getLength());
  }
  inline static uint32_t serializedLength(boost::call_traits<EuslispMessage>::param_type t) {
    return t.serializationLength();
  }
};
  }
}

/************************************************************
 *   Subscriptions
 ************************************************************/
class EuslispSubscriptionCallbackHelper : public ros::SubscriptionCallbackHelper {
public:
  pointer _scb,_args;
  EuslispMessage _msg;

  EuslispSubscriptionCallbackHelper(pointer scb, pointer args,pointer tmpl) :  _args(args), _msg(tmpl) {
    context *ctx = current_ctx;
    //ROS_WARN("func");prinx(ctx,scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,args,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    if (piscode(scb)) { // compiled code
      _scb=scb;
    } else if ((ccar(scb))==LAMCLOSURE) { // uncompiled code
      if ( ccar(ccdr(scb)) != NIL ) { // function
        _scb=ccar(ccdr(scb));
      } else { // lambda function
        _scb=scb;
      }
    } else {
      ROS_ERROR("subscription callback function install error");
    }
    // avoid gc
    pointer p=gensym(ctx);
    setval(ctx,intern(ctx,(char*)(p->c.sym.pname->c.str.chars),strlen((char*)(p->c.sym.pname->c.str.chars)),lisppkg),cons(ctx,scb,args));
  }
  ~EuslispSubscriptionCallbackHelper() {
      ROS_ERROR("subscription gc");
  }
  virtual ros::VoidConstPtr deserialize(const ros::SubscriptionCallbackHelperDeserializeParams& param) {
#if DEBUG
    cerr << __PRETTY_FUNCTION__ << endl;
    cerr << "param.length = " << param.length << endl;
    cerr << "param.buffer = " << (param.buffer + 4) << endl;
    cerr << "c_header == " << param.connection_header << endl;
    for(map<string, string>::iterator it = param.connection_header->begin(); it != param.connection_header->end(); it++){
      cerr << "            " << it->first << " : " << it->second << endl;
    }
#endif
    ros::VoidConstPtr ptr(new EuslispMessage(_msg));
    EuslispMessage *eus_msg = (EuslispMessage *)(ptr.get());
    eus_msg->deserialize(param.buffer, param.length);

    eus_msg->_connection_header = param.connection_header;
    return ptr;
  }
  virtual void call(ros::SubscriptionCallbackHelperCallParams& param) {
    EuslispMessage* eus_msg = (EuslispMessage *)((void *)param.event.getConstMessage().get());
    context *ctx = current_ctx;
    pointer argp=_args;
    int argc=0;
    //ROS_WARN("func");prinx(ctx,_scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,argp,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    vpush(eus_msg->_message);    // to avoid GC
    if ( ! ( issymbol(_scb) || piscode(_scb) || ccar(_scb)==LAMCLOSURE ) ) {
      ROS_ERROR("%s : can't find callback function", __PRETTY_FUNCTION__);
    }

    // store connection header
    StoreConnectionHeader(eus_msg);
    
    while(argp!=NIL){ ckpush(ccar(argp)); argp=ccdr(argp); argc++;}
    vpush((pointer)(eus_msg->_message));argc++;
    
    ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),_scb,(pointer)(ctx->vsp-argc),NULL,argc);
    while(argc-->0)vpop();
    vpop();    // remove eus_msg._message
  }
  virtual const std::type_info& getTypeInfo() {
    return typeid(EuslispMessage);
  }
  virtual bool isConst(){
    return true;
  }
  virtual bool hasHeader(){
    return true;
  }
};

/************************************************************
 *   ServiceCall
 ************************************************************/
class EuslispServiceCallbackHelper : public ros::ServiceCallbackHelper {
public:
  pointer _scb, _args;
  EuslispMessage _req, _res;
  string md5, datatype, requestDataType, responseDataType,
    requestMessageDefinition, responseMessageDefinition;

  EuslispServiceCallbackHelper(pointer scb, pointer args, string smd5, string sdatatype, pointer reqclass, pointer resclass) : _args(args), _req(reqclass), _res(resclass), md5(smd5), datatype(sdatatype) {
    context *ctx = current_ctx;
    //ROS_WARN("func");prinx(ctx,scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,args,ERROUT);flushstream(ERROUT);terpri(ERROUT);

    if (piscode(scb)) { // compiled code
      _scb=scb;
    } else if ((ccar(scb))==LAMCLOSURE) {
      if ( ccar(ccdr(scb)) != NIL ) { // function
        _scb=ccar(ccdr(scb));
      } else { // lambda function
        _scb=scb;
      }
    } else  {
      ROS_ERROR("service callback function install error");
    }
    // avoid gc
    pointer p=gensym(ctx);
    setval(ctx,intern(ctx,(char*)(p->c.sym.pname->c.str.chars),strlen((char*)(p->c.sym.pname->c.str.chars)),lisppkg),cons(ctx,scb,args));

    requestDataType = _req.__getDataType();
    responseDataType = _res.__getDataType();
    requestMessageDefinition = _req.__getMessageDefinition();
    responseMessageDefinition = _res.__getMessageDefinition();
  }
  ~EuslispServiceCallbackHelper() { }

  virtual boost::shared_ptr<EuslispMessage> createRequest() { return boost::shared_ptr<EuslispMessage>(new EuslispMessage(_req)); }
  virtual boost::shared_ptr<EuslispMessage> createResponse() { return boost::shared_ptr<EuslispMessage>(new EuslispMessage(_res)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }
  virtual std::string getRequestDataType() { return requestDataType; }
  virtual std::string getResponseDataType() { return responseDataType; }
  virtual std::string getRequestMessageDefinition() { return requestMessageDefinition; }
  virtual std::string getResponseMessageDefinition() { return responseMessageDefinition; }

  virtual bool call(ros::ServiceCallbackHelperCallParams& params) {
#if DEBUG
    cerr << __PRETTY_FUNCTION__ << endl;
    cerr << "param.length = " << params.request.num_bytes << endl;
    cerr << "param.buffer = " << (params.request.message_start + 4) << endl;
    cerr << "c_header == " << params.connection_header << endl;
    for(map<string, string>::iterator it = params.connection_header->begin(); it != params.connection_header->end(); it++){
      cerr << "            " << it->first << " : " << it->second << endl;
    }
#endif
    context *ctx = current_ctx;
    pointer r, argp=_args;
    int argc=0;

    vpush(_res._message);       // _res._message
    vpush(_req._message);       // _res._message, _req._message

    if ( ! ( issymbol(_scb) || piscode(_scb) || ccar(_scb)==LAMCLOSURE ) ) {
      ROS_ERROR("%s : can't find callback function", __PRETTY_FUNCTION__);
    }
    // Deserialization
    EuslispMessage eus_msg(_req);
    vpush(eus_msg._message);    // _res._message, _req._message, eus_msg._message
    eus_msg.deserialize(params.request.message_start, params.request.num_bytes);

    // store connection header
    eus_msg._connection_header = params.connection_header;
    StoreConnectionHeader(&eus_msg);

    while(argp!=NIL){ ckpush(ccar(argp)); argp=ccdr(argp); argc++;}
    vpush((pointer) eus_msg._message);argc++;

    r = ufuncall(ctx, (ctx->callfp?ctx->callfp->form:NIL),
                 _scb, (pointer)(ctx->vsp-argc),
                 NULL, argc);
    while(argc-->0)vpop();// _res._message, _req._message, eus_msg._message
    vpush(r); // _res._message, _req._message, eus_msg._message, r, 
    // Serializaion
    EuslispMessage eus_res(_res);
    eus_res.replaceContents(r);
    // check return value is valid
    pointer ret_serialize_method, ret_class;
    if (ispointer(r)) {
      ret_serialize_method = (pointer)findmethod(ctx,K_ROSEUS_SERIALIZATION_LENGTH,classof(r),&ret_class); }
    if (!ispointer(r) || ret_serialize_method == NIL) {
      ROS_ERROR("you may not return valid value in service callback");
      vpop(); // _res._message, _req._message, eus_msg._message, r
      vpop(); // _res._message, _req._message, eus_msg._message
      vpop(); // _res._message, _req._message,
      vpop(); // _res._message
      return false;
    }
    vpush(eus_res._message);    // _res._message, _req._message, eus_msg._message, r, eus_res._message
    uint32_t serialized_length = eus_res.serializationLength();
    params.response.num_bytes = serialized_length + 5; // add 5 bytes of message header
    params.response.buf.reset (new uint8_t[params.response.num_bytes]);
    params.response.message_start = 0;

    // SerializedResponseMessage
    uint8_t *tmp = params.response.buf.get();
    *tmp++ = 1; // 1 byte of success services flag, now always set true
    *tmp++ = (uint8_t)((serialized_length >> 0) & 0xFF); // 4bytes of message length
    *tmp++ = (uint8_t)((serialized_length >> 8) & 0xFF);
    *tmp++ = (uint8_t)((serialized_length >> 16) & 0xFF);
    *tmp++ = (uint8_t)((serialized_length >> 24) & 0xFF);
    eus_res.serialize(tmp, 0);
    // store connection header
    eus_res._connection_header = params.connection_header;
    StoreConnectionHeader(&eus_res);
#if DEBUG
    cerr << "num bytes = " << params.response.num_bytes << endl;
    ROS_INFO("message_start =  %X",params.response.message_start);
    ROS_INFO("message_ptr =  %X",params.response.buf.get());
    tmp = params.response.buf.get();
    for(int i=0;i<params.response.num_bytes;i++){
      ROS_INFO("%X", tmp[i]);
    }
#endif
    vpop(); // _res._message, _req._message, eus_msg._message, r, eus_res._message
    vpop(); // _res._message, _req._message, eus_msg._message, r
    vpop(); // _res._message, _req._message, eus_msg._message
    vpop(); // _res._message, _req._message,
    vpop(); // _res._message
    return true;
  }
};

void roseusSignalHandler(int sig)
{
    // memoize for euslisp handler...
    context *ctx=euscontexts[thr_self()];
    ctx->intsig = sig;
}

/************************************************************
 *   EUSLISP functions
 ************************************************************/
pointer ROSEUS(register context *ctx,int n,pointer *argv)
{
  char name[256] = "";
  uint32_t options = 0;
  int cargc = 0;
  char *cargv[32];

  if( s_bInstalled ) {
    ROS_WARN("ROSEUS is already installed as %s", ros::this_node::getName().c_str());
    return (T);
  }

  ckarg(3);
  if (isstring(argv[0]))
    strncpy(name,(char *)(argv[0]->c.str.chars),255);
  else error(E_NOSTRING);
  options = ckintval(argv[1]);
  pointer p = argv[2];
  if (islist(p)) {
    while (1) {
      if (!iscons(p)) break;
      cargv[cargc]=(char *)((ccar(p))->c.str.chars);
      cargc++;
      p=ccdr(p);
    }
  } else error(E_NOSEQ);

  // convert invalid node name charactors to _, we assume it is '-'
  for (unsigned int i=0; i < strlen(name); i++)
    if ( ! (isalpha(name[i]) || isdigit(name[i])) ) name[i] = '_';

  K_ROSEUS_MD5SUM   = defkeyword(ctx,"MD5SUM-");
  K_ROSEUS_DATATYPE = defkeyword(ctx,"DATATYPE-");
  K_ROSEUS_DEFINITION = defkeyword(ctx,"DEFINITION-");
  K_ROSEUS_CONNECTION_HEADER = intern(ctx,"_CONNECTION-HEADER",18,findpkg(makestring("ROS",3)));
  K_ROSEUS_SERIALIZATION_LENGTH = defkeyword(ctx,"SERIALIZATION-LENGTH");
  K_ROSEUS_SERIALIZE   = defkeyword(ctx,"SERIALIZE");
  K_ROSEUS_DESERIALIZE = defkeyword(ctx,"DESERIALIZE");
  K_ROSEUS_GET  = defkeyword(ctx,"GET");
  K_ROSEUS_INIT = defkeyword(ctx,"INIT");
  K_ROSEUS_REQUEST  = defkeyword(ctx,"REQUEST");
  K_ROSEUS_RESPONSE = defkeyword(ctx,"RESPONSE");
  K_ROSEUS_GROUPNAME = defkeyword(ctx,"GROUPNAME");
  K_ROSEUS_ONESHOT = defkeyword(ctx,"ONESHOT");
  K_ROSEUS_LAST_EXPECTED = defkeyword(ctx,"LAST-EXPECTED");
  K_ROSEUS_LAST_REAL = defkeyword(ctx,"LAST-REAL");
  K_ROSEUS_CURRENT_EXPECTED = defkeyword(ctx,"CURRENT-EXPECTED");
  K_ROSEUS_CURRENT_REAL = defkeyword(ctx,"CURRENT-REAL");
  K_ROSEUS_LAST_DURATION = defkeyword(ctx,"LAST-DURATION");
  K_ROSEUS_SEC = defkeyword(ctx,"SEC");
  K_ROSEUS_NSEC = defkeyword(ctx,"NSEC");

  s_mapAdvertised.clear();
  s_mapSubscribed.clear();
  s_mapServiced.clear();
  s_mapTimered.clear();
  s_mapHandle.clear();

  /*
    set locale to none to let C RTL assume logging string can contain non-ascii characters.
    Refer: https://logging.apache.org/log4cxx/latest_stable/faq.html
  */
  setlocale(LC_ALL, "");

  /*
    force to flag ros::init_options::NoSigintHandler.
    In fact, this code make no sence, because we steals
    SIGINT handler by the following `signal'.
   */
  options |= ros::init_options::NoSigintHandler;

  /*
    clear ros::master::g_uri which is defined in ros::master::init in __roseus.
    this occurs if user set unix::setenv("ROS_MASTER_URI") between __roseus and
    ROSEUS.
   */
  if (!ros::master::g_uri.empty()) {
    if ( ros::master::g_uri != getenv("ROS_MASTER_URI") ) {
      ROS_WARN("ROS master uri will be changed!!, master uri %s, which is defineed previously is differ from current ROS_MASTE_URI(%s)", ros::master::g_uri.c_str(), getenv("ROS_MASTER_URI"));
      ros::master::g_uri.clear();
    }
  }
  try {
    ros::init(cargc, cargv, name, options);
  } catch (const ros::InvalidNameException &e) {
    ROS_ERROR("%s",e.what());
    error(E_MISMATCHARG);
    return(NIL);
  }

  s_node.reset(new ros::NodeHandle());
  s_rate.reset(new ros::Rate(50));

  s_bInstalled = true;

  // install signal handler for sigint. DO NOT call unix:signal after
  // ros::roseus
  signal(SIGINT, roseusSignalHandler);
  return (T);
}

pointer ROSEUS_CREATE_NODEHANDLE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string groupname;
  string ns;

  ckarg2(1, 2);
  // ;; arguments ;;
  // groupname [ namespace ]

  if (isstring(argv[0])) groupname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  if ( n > 1 ) {
    if(isstring(argv[1])) ns.assign((char *)get_string(argv[1]));
    else error(E_NOSTRING);
  }

  if( s_mapHandle.find(groupname) != s_mapHandle.end() ) {
    ROS_DEBUG("groupname %s is already used", groupname.c_str());
    return (NIL);
  }

  boost::shared_ptr<NodeHandle> hd;
  if ( n > 1 ) {
    hd = boost::shared_ptr<NodeHandle> (new NodeHandle(ns));
    s_mapHandle[groupname] = hd;
  } else {
    hd = boost::shared_ptr<NodeHandle>(new NodeHandle());
    s_mapHandle[groupname] = hd;
  }
  //add callbackqueue to hd
  hd->setCallbackQueue( new CallbackQueue() );

  return (T);
}

pointer ROSEUS_SPIN(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  while (ctx->intsig==0 && ros::ok()) {
    ros::spinOnce();
    s_rate->sleep();
  }
  return (NIL);
}

pointer ROSEUS_SPINONCE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  ckarg2(0, 1);
  // ;; arguments ;;
  // [ groupname ]

  if ( n > 0 ) {
    string groupname;
    if (isstring(argv[0])) groupname.assign((char *)get_string(argv[0]));
    else error(E_NOSTRING);

    map<string, boost::shared_ptr<NodeHandle > >::iterator it = s_mapHandle.find(groupname);
    if( it == s_mapHandle.end() ) {
      ROS_ERROR("Groupname %s is missing", groupname.c_str());
      return (T);
    }
    boost::shared_ptr<NodeHandle > hdl = (it->second);
    // spin this nodehandle
    ((CallbackQueue *)hdl->getCallbackQueue())->callAvailable();

    return (NIL);
  } else {
    ros::spinOnce();
    return (NIL);
  }
}

pointer ROSEUS_TIME_NOW(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  pointer timevec;
  ros::Time t = ros::Time::now();

  timevec=makevector(C_INTVECTOR,2);
  vpush(timevec);
  timevec->c.ivec.iv[0]=t.sec;
  timevec->c.ivec.iv[1]=t.nsec;
  vpop();
  return (timevec);
}

pointer ROSEUS_RATE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  numunion nu;
  ckarg(1);
  float timeout=ckfltval(argv[0]);
  s_rate.reset(new ros::Rate(timeout));
  return(T);
}

pointer ROSEUS_SLEEP(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  s_rate->sleep();
  return (T);
}

pointer ROSEUS_DURATION_SLEEP(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  numunion nu;
  ckarg(1);
  float sleep=ckfltval(argv[0]);
  ros::Duration(sleep).sleep();
  return(T);
}

pointer ROSEUS_OK(register context *ctx,int n,pointer *argv)
{
  if (ros::ok()) {
    return (T);
  } else {
    return (NIL);
  }
}


#define def_rosconsole_formatter(funcname, rosfuncname)         \
  pointer funcname(register context *ctx,int n,pointer *argv)   \
  { pointer *argv2,msg;                                         \
    int argc2;                                                  \
    argc2 = n+1;                                                \
    argv2 = (pointer *)malloc(sizeof(pointer)*argc2);           \
    argv2[0] = NIL;                                             \
    for(int i=0;i<n;i++) argv2[i+1]=argv[i] ;                   \
    msg = XFORMAT(ctx, argc2, argv2);                           \
    rosfuncname("%s", msg->c.str.chars);                        \
    free(argv2);                                                \
    return (T);                                                 \
  }

def_rosconsole_formatter(ROSEUS_ROSDEBUG, ROS_DEBUG)
def_rosconsole_formatter(ROSEUS_ROSINFO,  ROS_INFO)
def_rosconsole_formatter(ROSEUS_ROSWARN,  ROS_WARN)
def_rosconsole_formatter(ROSEUS_ROSERROR, ROS_ERROR)
def_rosconsole_formatter(ROSEUS_ROSFATAL, ROS_FATAL)

pointer ROSEUS_EXIT(register context *ctx,int n,pointer *argv)
{
  ROS_INFO("%s", __PRETTY_FUNCTION__);
  if( s_bInstalled ) {
    ROS_INFO("exiting roseus %ld", (n==0)?n:ckintval(argv[0]));
    s_mapAdvertised.clear();
    s_mapSubscribed.clear();
    s_mapServiced.clear();
    s_mapTimered.clear();
    s_mapHandle.clear();
    ros::shutdown();
  }
  if (n==0) _exit(0);
  else _exit(ckintval(argv[0]));
}

/************************************************************
 *   ROSEUS Publisher/Subscriber
 ************************************************************/
pointer ROSEUS_SUBSCRIBE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string topicname;
  pointer message, fncallback, args;
  int queuesize = 1;
  NodeHandle *lnode = s_node.get();

  // ;; arguments ;;
  // topicname message_type callbackfunc args0 ... argsN [ queuesize ] [ :groupname groupname ]
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  if (n > 1 && issymbol(argv[n-2]) && isstring(argv[n-1])) {
    if (argv[n-2] == K_ROSEUS_GROUPNAME) {
      string groupname;
      groupname.assign((char *)get_string(argv[n-1]));
      map<string, boost::shared_ptr<NodeHandle > >::iterator it = s_mapHandle.find(groupname);
      if( it != s_mapHandle.end() ) {
        ROS_DEBUG("subscribe with groupname=%s", groupname.c_str());
        lnode = (it->second).get();
      } else {
        ROS_ERROR("Groupname %s is missing. Topic %s is not subscribed. Call (ros::create-nodehandle \"%s\") first.",
                  groupname.c_str(), topicname.c_str(), groupname.c_str());
        return (NIL);
      }
      n -= 2;
    }
  }
  if (isint(argv[n-1])) {queuesize = ckintval(argv[n-1]);n--;}
  ROS_DEBUG("subscribe %s queuesize=%d", topicname.c_str(), queuesize);
  // TODO:need error checking
  message = argv[1];
  fncallback = argv[2];
  args=NIL;
  for (int i=n-1;i>=3;i--) args=cons(ctx,argv[i],args);

  EuslispMessage msg(message);
   boost::shared_ptr<SubscriptionCallbackHelper> *callback =
     (new boost::shared_ptr<SubscriptionCallbackHelper>
      (new EuslispSubscriptionCallbackHelper(fncallback, args, message)));
  SubscribeOptions so(topicname, queuesize, msg.__getMD5Sum(), msg.__getDataType());
  so.helper = *callback;
  Subscriber subscriber = lnode->subscribe(so);
  boost::shared_ptr<Subscriber> sub = boost::shared_ptr<Subscriber>(new Subscriber(subscriber));
  if ( !!sub ) {
    s_mapSubscribed[topicname] = sub;
  } else {
    ROS_ERROR("s_mapSubscribed");
  }

  return (T);
}

pointer ROSEUS_UNSUBSCRIBE(register context *ctx,int n,pointer *argv)
{
  string topicname;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = s_mapSubscribed.erase(topicname)>0;

  return (bSuccess?T:NIL);
}

pointer ROSEUS_GETNUMPUBLISHERS(register context *ctx,int n,pointer *argv)
{
  string topicname;
  int ret;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Subscriber> >::iterator it = s_mapSubscribed.find(topicname);
  if( it != s_mapSubscribed.end() ) {
    boost::shared_ptr<Subscriber> subscriber = (it->second);
    ret = subscriber->getNumPublishers();
    bSuccess = true;
  }

  return (bSuccess?(makeint(ret)):NIL);
}

pointer ROSEUS_GETTOPICSUBSCRIBER(register context *ctx,int n,pointer *argv)
{
  string topicname;
  string ret;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Subscriber> >::iterator it = s_mapSubscribed.find(topicname);
  if( it != s_mapSubscribed.end() ) {
    boost::shared_ptr<Subscriber> subscriber = (it->second);
    ret = subscriber->getTopic();
    bSuccess = true;
  }

  return (bSuccess?(makestring((char *)ret.c_str(), ret.length())):NIL);
}

pointer ROSEUS_ADVERTISE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string topicname;
  pointer message;
  int queuesize = 1;
  bool latch = false;

  ckarg2(2,4);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  message = argv[1];
  if ( n > 2 ) {
    queuesize = ckintval(argv[2]);
  }
  if ( n > 3 ) {
    latch = (argv[3]!=NIL ? true : false);
  }
  ROS_DEBUG("advertise %s %d %d", topicname.c_str(), queuesize, latch);
  if( s_mapAdvertised.find(topicname) != s_mapAdvertised.end() ) {
    ROS_WARN("topic %s already advertised", topicname.c_str());
    return (NIL);
  }

  EuslispMessage msg(message);
  AdvertiseOptions ao(topicname, queuesize, msg.__getMD5Sum(), msg.__getDataType(), msg.__getMessageDefinition());
  ao.latch = latch;
  Publisher publisher = s_node->advertise(ao);
  boost::shared_ptr<Publisher> pub = boost::shared_ptr<Publisher>(new Publisher(publisher));
  if ( !!pub ) {
    s_mapAdvertised[topicname] = pub;
  } else {
    ROS_ERROR("s_mapAdvertised");
  }

  return (T);
}

pointer ROSEUS_UNADVERTISE(register context *ctx,int n,pointer *argv)
{
  string topicname;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = s_mapAdvertised.erase(topicname)>0;

  return (bSuccess?T:NIL);
}

pointer ROSEUS_PUBLISH(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string topicname;
  pointer emessage;

  ckarg(2);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  emessage = argv[1];

  bool bSuccess = false;
  map<string, boost::shared_ptr<Publisher> >::iterator it = s_mapAdvertised.find(topicname);
  if( it != s_mapAdvertised.end() ) {
    boost::shared_ptr<Publisher> publisher = (it->second);
    EuslispMessage message(emessage);
    publisher->publish(message);
    bSuccess = true;
  }

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to publish to topic %s, which was not " \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_GETNUMSUBSCRIBERS(register context *ctx,int n,pointer *argv)
{
  string topicname;
  int ret;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Publisher> >::iterator it = s_mapAdvertised.find(topicname);
  if( it != s_mapAdvertised.end() ) {
    boost::shared_ptr<Publisher> publisher = (it->second);
    ret = publisher->getNumSubscribers();
    bSuccess = true;
  }

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to getNumSubscribers to topic %s, which was not " \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (bSuccess?(makeint(ret)):NIL);
}

pointer ROSEUS_GETTOPICPUBLISHER(register context *ctx,int n,pointer *argv)
{
  string topicname;
  string ret;

  ckarg(1);
  if (isstring(argv[0])) topicname = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Publisher> >::iterator it = s_mapAdvertised.find(topicname);
  if( it != s_mapAdvertised.end() ) {
    boost::shared_ptr<Publisher> publisher = (it->second);
    ret = publisher->getTopic();
    bSuccess = true;
  }

  return (bSuccess?(makestring((char *)ret.c_str(), ret.length())):NIL);
}

/************************************************************
 *   ROSEUS ServiceCall
 ************************************************************/
pointer ROSEUS_WAIT_FOR_SERVICE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string service;

  ckarg2(1,2);
  if (isstring(argv[0])) service = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  int32_t timeout = -1;

  if( n > 1 )
    timeout = (int32_t)ckintval(argv[1]);

  bool bSuccess = service::waitForService(service, ros::Duration(timeout));

  return (bSuccess?T:NIL);
}

pointer ROSEUS_SERVICE_EXISTS(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string service;

  ckarg(1);
  if (isstring(argv[0])) service = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = service::exists(service, true);

  return (bSuccess?T:NIL);
}

pointer ROSEUS_SERVICE_CALL(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string service;
  pointer emessage;
  bool persist = false;
  ckarg2(2,3);
  if (isstring(argv[0])) service = ros::names::resolve((char *)get_string(argv[0]));
  else  error(E_NOSTRING);
  emessage = argv[1];
  if ( n > 2 ) {
      persist = (argv[2] != NIL ? true : false);
  }
  static std::map<std::string, ros::ServiceServerLinkPtr> service_link_cache;
  static std::map<std::string, std::string> service_md5_cache;
  EuslispMessage request(emessage);
  vpush(request._message);      // to avoid GC, it may not be required...
  EuslispMessage response(csend(ctx,emessage,K_ROSEUS_RESPONSE,0));
  vpush(response._message);     // to avoid GC, its important

  // NOTE: To make use of connection header, ServiceServerLink is manually handled instead of just invoking ServiceClient.call.
  // ServiceClientOptions sco(service, request.__getMD5Sum(), persist, M_string());
  // client = s_node->serviceClient(sco);
  // bool bSuccess =  client.call(request, response, request.__getMD5Sum());
  ros::ServiceServerLinkPtr link;
  if (persist) {
    if ( service_link_cache.find(service) != service_link_cache.end() &&
         ( ! service_link_cache[service]->isValid() ||
           service_md5_cache[service] !=request.__getMD5Sum() )) {
      service_link_cache[service]->getConnection()->drop(ros::Connection::Destructing);
      service_link_cache[service].reset();
      service_link_cache.erase(service);
      service_md5_cache.erase(service);
    }

    if (service_link_cache.find(service) == service_link_cache.end()) {
      service_link_cache[service] = ros::ServiceManager::instance()->createServiceServerLink(service, persist, request.__getMD5Sum(), request.__getMD5Sum(), M_string());
      service_md5_cache[service] = request.__getMD5Sum();
    }
    link = service_link_cache[service];
  } else {
    link = ros::ServiceManager::instance()->createServiceServerLink(service, persist, request.__getMD5Sum(), request.__getMD5Sum(), M_string());
  }

  bool bSuccess = true;
  ros::SerializedMessage ser_req = ros::serialization::serializeMessage(request);
  ros::SerializedMessage ser_resp;
  if (link && link->isValid()) {
    bSuccess = link->call(ser_req, ser_resp);
    if ( bSuccess ) {
      try {
        ros::serialization::deserializeMessage(ser_resp, response);
      } catch (std::exception& e) {
        ROS_ERROR("Exception thrown while deserializing service call: %s", e.what());
        bSuccess = false;
      }
      boost::shared_ptr<map<string, string> >connection_header = link->getConnection()->getHeader().getValues();
#if DEBUG
      cerr << __PRETTY_FUNCTION__ << endl;
      cerr << "c_header == " << connection_header << endl;
      for(map<string, string>::iterator it = connection_header->begin(); it != connection_header->end(); it++){
        cerr << "            " << it->first << " : " << it->second << endl;
      }
#endif
      response._connection_header = connection_header;
      StoreConnectionHeader(&response);
      link.reset();
      // If we're shutting down but the node haven't finished yet, wait until we do
      while (ros::isShuttingDown() && ros::ok()) {
        ros::WallDuration(0.001).sleep();
      }
    }
  } else {
    bSuccess = false;
    ROS_ERROR("Failed to establish connection to service server");
  }
  vpop();                       // pop response._message
  vpop();                       // pop request._message
  if ( ! bSuccess ) {
    ROS_ERROR("attempted to call service  %s, but failed ",
              ros::names::resolve(service).c_str());
  }

  return (response._message);
}

pointer ROSEUS_ADVERTISE_SERVICE(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  string service;
  pointer emessage;
  pointer fncallback, args;

  if (isstring(argv[0])) service = ros::names::resolve((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  emessage = argv[1];
  fncallback = argv[2];
  args=NIL;
  for (int i=n-1;i>=3;i--) args=cons(ctx,argv[i],args);
  if( s_mapServiced.find(service) != s_mapServiced.end() ) {
    ROS_INFO("service %s already advertised", service.c_str());
    return (NIL);
  }

  EuslispMessage message(emessage);
  vpush(message._message);      // to avoid GC in csend
  pointer request(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_REQUEST));
  pointer response(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_RESPONSE));
  vpop();                       // pop message._message
  boost::shared_ptr<EuslispServiceCallbackHelper> *callback =
    (new boost::shared_ptr<EuslispServiceCallbackHelper>
     (new EuslispServiceCallbackHelper(fncallback, args, message.__getMD5Sum(),
                                       message.__getDataType(), request, response)));
  AdvertiseServiceOptions aso;
  aso.service.assign(service);
  aso.datatype = (*callback->get()).getDataType();
  aso.md5sum = (*callback->get()).getMD5Sum();
  aso.req_datatype = (*callback->get()).getRequestDataType();
  aso.res_datatype = (*callback->get()).getResponseDataType();
  aso.helper = *callback;
  ServiceServer server = s_node->advertiseService(aso);
  boost::shared_ptr<ServiceServer> ser = boost::shared_ptr<ServiceServer>(new ServiceServer(server));
  if ( !!ser ) {
    s_mapServiced[service] = ser;
  } else {
    ROS_ERROR("s_mapServiced");
  }

  return (T);
}

pointer ROSEUS_UNADVERTISE_SERVICE(register context *ctx,int n,pointer *argv)
{
  string service;

  ckarg(1);
  if (isstring(argv[0])) service = ros::names::resolve((char *)(argv[0]));
  else error(E_NOSTRING);

  ROS_DEBUG("unadvertise %s", service.c_str());
  bool bSuccess = s_mapServiced.erase(service)>0;

  return (bSuccess?T:NIL);
}

void EusValueToXmlRpc(register context *ctx, pointer argp, XmlRpc::XmlRpcValue& rpc_value)
{
  numunion nu;

  if ( islist(argp) && islist(ccar(argp))) { // alist
    pointer a;
    int j;
    // set keys
    std::ostringstream stringstream;
    stringstream << "<value><struct>";
    a = argp;
    while(islist(a)){
      pointer v = ccar(a);
      if ( iscons(v) ) { // is alist
        if ( issymbol(ccar(v)) ) {
          string skey = string((char *)get_string(ccar(v)->c.sym.pname));
          boost::algorithm::to_lower(skey);
          stringstream << "<member><name>" << skey << "</name><value><boolean>0</boolean></value></member>";
        }else{
          ROS_ERROR("ROSEUS_SET_PARAM: EusValueToXmlRpc: assuming symbol");prinx(ctx,ccar(v),ERROUT);flushstream(ERROUT);terpri(ERROUT);
        }
      }else{
        ROS_ERROR("ROSEUS_SET_PARAM: EusValueToXmlRpc: assuming alist");prinx(ctx,argp,ERROUT);flushstream(ERROUT);terpri(ERROUT);
      }
      a=ccdr(a);
    }
    stringstream << "</struct></value>";
    j=0; rpc_value.fromXml(stringstream.str(), &j);
    // set values
    a = argp;
    while(islist(a)){
      pointer v = ccar(a);
      if ( iscons(v) ) {
        if ( issymbol(ccar(v)) ) {
          string skey = string((char *)get_string(ccar(v)->c.sym.pname));
          boost::algorithm::to_lower(skey);
          XmlRpc::XmlRpcValue p;
          EusValueToXmlRpc(ctx, ccdr(v), p);
          rpc_value[skey] = p;
        }
      }
      a=ccdr(a);
    }
  } else if ( islist(argp) ) { // list
    pointer a;
    int i;
    // get size
    a = argp;
    i = 0; while ( islist(a) ) { a=ccdr(a); i++; }
    rpc_value.setSize(i);
    // fill items
    a = argp;
    i = 0;
    while ( islist(a) ) {
      XmlRpc::XmlRpcValue p;
      EusValueToXmlRpc(ctx, ccar(a), p);
      rpc_value[i] = p;
      a=ccdr(a);
      i++;
    }
  } else if ( isstring(argp) ) {
    rpc_value = (char *)get_string(argp);
  } else if ( isint(argp) ) {
    rpc_value = (int)(intval(argp));
  } else if  ( isflt(argp) ) {
    rpc_value = (double)(fltval(argp));
  } else if (argp == T) {
    rpc_value = XmlRpc::XmlRpcValue((bool)true);
  } else if (argp == NIL) {
    rpc_value = XmlRpc::XmlRpcValue((bool)false);
  } else if ( issymbol(argp) ) {
    string s((char *)get_string(argp->c.sym.pname));
    boost::algorithm::to_lower(s);
    rpc_value = s.c_str();
  } else {
    ROS_ERROR("EusValueToXmlRpc: unknown parameters");prinx(ctx,argp,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    error(E_MISMATCHARG);
  }
}

pointer ROSEUS_SET_PARAM(register context *ctx,int n,pointer *argv)
{
  string key;
  string s;

  ckarg(2);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  XmlRpc::XmlRpcValue param;
  EusValueToXmlRpc(ctx, argv[1], param);
  ros::param::set(key,param);

  return (T);
}

pointer XmlRpcToEusValue(register context *ctx, XmlRpc::XmlRpcValue rpc_value)
{
  numunion nu;
  pointer ret, first;

  if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean ){
    if ( rpc_value ) return T; else return NIL;
  }
  else if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeDouble ){
    return makeflt((double)rpc_value);
  }
  else if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeInt ){
    return makeint((int)rpc_value);
  }
  else if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeString ){
    std::string str = rpc_value;
    return makestring((char*)str.c_str(), ((std::string)rpc_value).length());
  }
  else if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray ){
    ret = cons(ctx, NIL, NIL);
    first = ret;
    vpush(ret);
    for ( int i = 0; i < rpc_value.size(); i++){
      ccdr(ret) = cons(ctx, XmlRpcToEusValue(ctx, rpc_value[i]), NIL);
      ret = ccdr(ret);
    }
    vpop(); // vpush(ret);
    return ccdr(first);
  }
  else if ( rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct ){
    ret = cons(ctx, NIL, NIL);
    first = ret;
    vpush(ret);
    XmlRpc::XmlRpcValue::iterator it = rpc_value.begin();
    while(it !=rpc_value.end()) {
      std::string key = it->first;
      pointer tmp = cons(ctx, makestring((char*)key.c_str(), key.length()), NIL);
      vpush(tmp);
      ccdr(tmp) = XmlRpcToEusValue(ctx, it->second);
      ccdr(ret) = cons(ctx, tmp, NIL);
      ret = ccdr(ret);
      vpop(); // vpush(tmp);
      it++;
    }
    vpop(); // vpush(ret);
    return ccdr(first);
  } else {
    ROS_FATAL("unknown rosparam type!");
    return NIL;
  }
  return NIL;
}

pointer XmlRpcToEusList(register context *ctx, XmlRpc::XmlRpcValue param_list)
{
    numunion nu;
    pointer ret, first;
    ret = cons(ctx, NIL, NIL);
    first = ret;
    vpush(ret);
    if ( param_list.getType() == XmlRpc::XmlRpcValue::TypeArray ){
        for ( int i = 0; i < param_list.size(); i++){
            if ( param_list[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean ){
                if ( param_list[i] ){
                    ccdr(ret) = cons(ctx, T, NIL);
                    ret = ccdr(ret);
                } else {
                    ccdr(ret) = cons(ctx, NIL, NIL);
                    ret = ccdr(ret);
                }
            }
            else if ( param_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ){
                ccdr(ret) = cons(ctx, makeflt((double)param_list[i]), NIL);
                ret = ccdr(ret);
            }
            else if ( param_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt ){
                ccdr(ret) = cons(ctx, makeint((int)param_list[i]), NIL);
                ret = ccdr(ret);
            }
            else if ( param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString ){
                std::string str = param_list[i];
                ccdr(ret) = cons(ctx, makestring((char*)str.c_str(), ((std::string)param_list[i]).length()), NIL);
                ret = ccdr(ret);
            }
            else if ( param_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct ){
                ccdr(ret) = cons(ctx, XmlRpcToEusValue(ctx, param_list[i]), NIL);
                ret = ccdr(ret);
            }
            else {
                ROS_FATAL("unknown rosparam type!");
                vpop();         // remove vpush(ret)
                return NIL;
            }
        }
        vpop();                 // remove vpush(ret)
        return ccdr(first);
    } else if ( param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct ) {
        return XmlRpcToEusValue(ctx, param_list);
    } else
        return (NIL);
}

pointer ROSEUS_GET_PARAM(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;

  ckarg2(1,2);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  string s;
  double d;
  bool b;
  int i;
  pointer ret;
  XmlRpc::XmlRpcValue param_list;

  if ( ros::param::get(key, s) ) {
    ret = makestring((char *)s.c_str(), s.length());
  } else if ( ros::param::get(key, d) ) {
    ret = makeflt(d);
  } else if ( ros::param::get(key, i) ) {
    ret = makeint(i);
  } else if ( ros::param::get(key, b) ) {
      if ( b == true )
          ret = T;
      else
          ret = NIL;
  } else if (ros::param::get(key, param_list)){
      ret = XmlRpcToEusList(ctx, param_list);
  }else {
    if ( n == 2 ) {
      ret = COPYOBJ(ctx,1,argv+1);
    } else {
      ROS_ERROR("unknown ros::param::get, key=%s", key.c_str());
      ret = NIL;
    }
  }
  return (ret);
}

pointer ROSEUS_GET_PARAM_CACHED(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;

  ckarg2(1,2);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  string s;
  double d;
  int i;
  bool b;
  pointer ret;
  XmlRpc::XmlRpcValue param_list;
  if ( ros::param::getCached(key, s) ) {
    ret = makestring((char *)s.c_str(), s.length());
  } else if ( ros::param::getCached(key, d) ) {
    ret = makeflt(d);
  } else if ( ros::param::getCached(key, i) ) {
    ret = makeint(i);
  } else if ( ros::param::getCached(key, b) ) {
      if ( b == true )
          ret = T;
      else
          ret = NIL;
  } else if (ros::param::getCached(key, param_list)){
      ret = XmlRpcToEusList(ctx, param_list);
  } else {
    if ( n == 2 ) {
      ret = COPYOBJ(ctx,1,argv+1);
    } else {
      ROS_ERROR("unknown ros::param::get, key=%s", key.c_str());
      ret = NIL;
    }
  }
  return (ret);
}

pointer ROSEUS_HAS_PARAM(register context *ctx,int n,pointer *argv)
{
  string key;

  ckarg(1);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  return((ros::param::has(key))?(T):(NIL));
}

pointer ROSEUS_DELETE_PARAM(register context *ctx,int n,pointer *argv)
{
  string key;

  ckarg(1);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  return((ros::param::del(key))?(T):(NIL));
}

pointer ROSEUS_SEARCH_PARAM(register context *ctx,int n,pointer *argv)
{
  string key, result;

  ckarg(1);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  if ( ros::param::search(key, result) ) {
    return makestring((char *)result.c_str(), result.length());
  }
  return(NIL);
}

pointer ROSEUS_LIST_PARAM(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

#if ROS_VERSION_MINIMUM(1,11,17)
  std::vector<std::string> keys;
  if ( ros::param::getParamNames(keys) ) {
    pointer ret = cons(ctx, NIL,NIL), first;
    first = ret;
    vpush(ret);
    for(std::vector<std::string>::iterator it = keys.begin(); it != keys.end(); it++) {
      const std::string& key = *it;
      ccdr(ret) = cons(ctx, makestring((char *)key.c_str(), key.length()), NIL);
      ret = ccdr(ret);
    }
    vpop();
    return ccdr(first);
  }
#else
  ROS_ERROR("%s : ros::rosparam::getParamNames is not implemented for roscpp %d.%d.%d",  __PRETTY_FUNCTION__, ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH);
#endif
  return(NIL);
}

pointer ROSEUS_ROSPACK_FIND(register context *ctx,int n,pointer *argv)
{
  ckarg(1);

  string pkg;
  if (isstring(argv[0])) pkg.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  try {
#ifdef ROSPACK_EXPORT
      rospack::Package *p = rp.get_pkg(pkg);
      if (p!=NULL) return(makestring((char *)p->path.c_str(),p->path.length()));
#else
      std::string path;
      if (rp.find(pkg,path)==true) return(makestring((char *)path.c_str(),path.length()));
#endif
  } catch (runtime_error &e) {
  }
  return(NIL);
}

pointer ROSEUS_ROSPACK_DEPENDS(register context *ctx,int n,pointer *argv)
{
  // (ros::rospack-depends package-name)
  ckarg(1);

  string pkg;
  if (isstring(argv[0])) pkg.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  try {
      // not sure why need this, otherwise failed in ImportError: /usr/lib/python2.7/lib-dynload/_elementtree.x86_64-linux-gnu.so: undefined symbol: PyExc_RuntimeError
      std::vector<std::string> flags;
      std::vector<rospack::Stackage*> stackages;
      if(!rp.depsOnDetail(pkg, true, stackages, true))
        return (NIL);
      //
      std::vector<std::string> deps;
      if (rp.deps(pkg,false,deps)) {
        register pointer ret, first;
        ret=cons(ctx, NIL, NIL);
        first = ret;
        vpush(ret);
        for (std::vector<std::string>::iterator it = deps.begin() ; it != deps.end(); it++) {
          const std::string& dep = *it;
          ccdr(ret) = cons(ctx, makestring((char *)dep.c_str(), dep.length()), NIL);
          ret = ccdr(ret);
        }
        vpop(); // vpush(ret)
        return ccdr(first);
      }
  } catch (runtime_error &e) {
  }
  return(NIL);
}

pointer ROSEUS_ROSPACK_PLUGINS(register context *ctx,int n,pointer *argv)
{
  // (ros::rospack-plugins package-name attibute-name)
  ckarg(2);
  string pkg, attrib;
  pointer ret, first;
  if (isstring(argv[0])) pkg.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  if (isstring(argv[1])) attrib.assign((char *)get_string(argv[1]));
  else error(E_NOSTRING);
  try {
      std::vector<std::string> flags;
      if (rp.plugins(pkg, attrib, "", flags)) {
          ret = cons(ctx, NIL, NIL);
          first = ret;
          vpush(ret);
          for (size_t i = 0; i < flags.size(); i++) {
            // flags[i] = laser_proc /opt/ros/hydro/share/laser_proc/nodelets.xml
            std::vector<std::string> parsed_string;
            boost::algorithm::split(parsed_string, flags[i], boost::is_any_of(" "));
            std::string package = parsed_string[0];
            std::string value = parsed_string[1];
            pointer tmp = cons(ctx, cons(ctx,
                                         makestring((char*)package.c_str(), package.length()),
                                         makestring((char*)value.c_str(), value.length())),
                               NIL);
            ccdr(ret) = tmp;
            ret = tmp;
          }
          vpop();            // ret
          return ccdr(first);
      }
      else {
          return(NIL);
      }
  }
  catch (runtime_error &e) {
  }
  return(NIL);
}

pointer ROSEUS_RESOLVE_NAME(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  if (!isstring(argv[0])) error(E_NOSTRING);
  std::string src;
  src.assign((char *)(argv[0]->c.str.chars));
  std::string dst = ros::names::resolve(src);
  return(makestring((char *)dst.c_str(), dst.length()));
}

pointer ROSEUS_GETNAME(register context *ctx,int n,pointer *argv)
{
  ckarg(0);
  return(makestring((char *)ros::this_node::getName().c_str(),
		    ros::this_node::getName().length()));
}

pointer ROSEUS_GETNAMESPACE(register context *ctx,int n,pointer *argv)
{
  ckarg(0);
  // https://github.com/ros/ros_comm/pull/1100
  // Clean the namespace to get rid of double or trailing forward slashes
  std::string ns  = ros::names::clean(ros::this_node::getNamespace()).c_str();
  return(makestring((char *)ns.c_str(), ns.length()));
}

pointer ROSEUS_SET_LOGGER_LEVEL(register context *ctx, int n, pointer *argv)
{
  ckarg(2);
  string logger;
  if (isstring(argv[0])) logger.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  int log_level = intval(argv[1]);
  ros::console::levels::Level  level = ros::console::levels::Debug;
  switch(log_level){
  case 1:
    level = ros::console::levels::Debug;
    break;
  case 2:
    level = ros::console::levels::Info;
    break;
  case 3:
    level = ros::console::levels::Warn;
    break;
  case 4:
    level = ros::console::levels::Error;
    break;
  case 5:
    level = ros::console::levels::Fatal;
    break;
  default:
    return (NIL);
  }

  bool success = ::ros::console::set_logger_level(logger, level);
  if (success)
    {
      console::notifyLoggerLevelsChanged();
      return (T);
    }
  return (NIL);
}

pointer ROSEUS_GET_HOST(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

  std::string host = ros::master::getHost();
  return makestring((char*)host.c_str(), host.length());
}

pointer ROSEUS_GET_NODES(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

  ros::V_string nodes;
  if ( ! ros::master::getNodes(nodes) ) {
    return NIL;
  }

  register pointer ret, first;
  ret=cons(ctx, NIL, NIL);
  first = ret;
  vpush(ret);
  for (ros::V_string::iterator it = nodes.begin() ; it != nodes.end(); it++) {
    std::string node = *it;
    ccdr(ret) = cons(ctx, makestring((char *)node.c_str(), node.length()), NIL);
    ret = ccdr(ret);
  }
  vpop(); // vpush(ret)

  return ccdr(first);
}

pointer ROSEUS_GET_PORT(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

  return makeint(ros::master::getPort());
}

pointer ROSEUS_GET_URI(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

  std::string uri = ros::master::getURI();
  return makestring((char*)uri.c_str(), uri.length());
}

pointer ROSEUS_GET_TOPICS(register context *ctx,int n,pointer *argv)
{
  ckarg(0);

  ros::master::V_TopicInfo topics;
  if ( !ros::master::getTopics(topics) ) {
    return NIL;
  }

  register pointer ret, first;
  ret=cons(ctx, NIL, NIL);
  first = ret;
  vpush(ret);
  for (ros::master::V_TopicInfo::iterator it = topics.begin() ; it != topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    pointer tmp = cons(ctx,makestring((char*)info.name.c_str(), info.name.length()), makestring((char*)info.datatype.c_str(), info.datatype.length()));
    vpush(tmp);
    ccdr(ret) = cons(ctx, tmp, NIL);
    ret = ccdr(ret);
    vpop(); // vpush(tmp)
  }
  vpop(); // vpush(ret)

  return ccdr(first);
}

class TimerFunction
{
  pointer _scb, _args;
public:
  TimerFunction(pointer scb, pointer args) : _scb(scb), _args(args) {
    //context *ctx = current_ctx;
    //ROS_WARN("func");prinx(ctx,scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,args,ERROUT);flushstream(ERROUT);terpri(ERROUT);
  }
  void operator()(const ros::TimerEvent& event)
  {
    context *ctx = current_ctx;
    pointer argp=_args;
    int argc=0;

    pointer clsptr = NIL;
    for(int i=0; i<nextcix;i++) {
      if(!memcmp(classtab[i].def->c.cls.name->c.sym.pname->c.str.chars,(char *)"TIMER-EVENT",11)) {
        clsptr = classtab[i].def;
      }
    }
    if ( ! ( issymbol(_scb) || piscode(_scb) || ccar(_scb)==LAMCLOSURE ) ) {
      ROS_ERROR("%s : can't find callback function", __PRETTY_FUNCTION__);
    }

    pointer tevent = makeobject(clsptr);
    csend(ctx,tevent,K_ROSEUS_INIT,0);
    csend(ctx,tevent,K_ROSEUS_LAST_EXPECTED,2,K_ROSEUS_SEC,makeint(event.last_expected.sec));
    csend(ctx,tevent,K_ROSEUS_LAST_EXPECTED,2,K_ROSEUS_NSEC,makeint(event.last_expected.nsec));
    csend(ctx,tevent,K_ROSEUS_LAST_REAL,2,K_ROSEUS_SEC,makeint(event.last_real.sec));
    csend(ctx,tevent,K_ROSEUS_LAST_REAL,2,K_ROSEUS_NSEC,makeint(event.last_real.nsec));
    csend(ctx,tevent,K_ROSEUS_CURRENT_EXPECTED,2,K_ROSEUS_SEC,makeint(event.current_expected.sec));
    csend(ctx,tevent,K_ROSEUS_CURRENT_EXPECTED,2,K_ROSEUS_NSEC,makeint(event.current_expected.nsec));
    csend(ctx,tevent,K_ROSEUS_CURRENT_REAL,2,K_ROSEUS_SEC,makeint(event.current_real.sec));
    csend(ctx,tevent,K_ROSEUS_CURRENT_REAL,2,K_ROSEUS_NSEC,makeint(event.current_real.nsec));
    csend(ctx,tevent,K_ROSEUS_LAST_DURATION,2,K_ROSEUS_SEC,makeint(event.profile.last_duration.sec));
    csend(ctx,tevent,K_ROSEUS_LAST_DURATION,2,K_ROSEUS_NSEC,makeint(event.profile.last_duration.nsec));

    while(argp!=NIL){ ckpush(ccar(argp)); argp=ccdr(argp); argc++;}
    vpush((pointer)(tevent));argc++;

    ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),_scb,(pointer)(ctx->vsp-argc),NULL,argc);
    while(argc-->0)vpop();

  }
};

pointer ROSEUS_CREATE_TIMER(register context *ctx,int n,pointer *argv)
{
  isInstalledCheck;
  numunion nu;
  bool oneshot = false;
  pointer fncallback = NIL, args;
  NodeHandle *lnode = s_node.get();
  string fncallname;
  float period=ckfltval(argv[0]);

  // period callbackfunc args0 ... argsN [ oneshot ]
  // ;; oneshot ;;
  if (n > 1 && issymbol(argv[n-2]) && issymbol(argv[n-1])) {
    if (argv[n-2] == K_ROSEUS_ONESHOT) {
      if ( argv[n-1] != NIL ) {
        oneshot = true;
      }
      n -= 2;
    }
  }
  // ;; functions ;;
  if (piscode(argv[1])) { // compiled code
    fncallback=argv[1];
    std::ostringstream stringstream;
    stringstream << reinterpret_cast<long>(argv[2]) << " ";
    for (int i=3; i<n;i++)
      stringstream << string((char*)(argv[i]->c.sym.pname->c.str.chars)) << " ";
    fncallname = stringstream.str();
  } else if ((ccar(argv[1]))==LAMCLOSURE) { // uncompiled code
    if ( ccar(ccdr(argv[1])) != NIL ) { // function
      fncallback=ccar(ccdr(argv[1]));
      fncallname = string((char*)(fncallback->c.sym.pname->c.str.chars));
    } else { // lambda function
      fncallback=argv[1];
      std::ostringstream stringstream;
      stringstream << reinterpret_cast<long>(argv[1]);
      fncallname = stringstream.str();
    }
  } else {
    ROS_ERROR("subscription callback function install error");
  }

  // ;; arguments ;;
  args=NIL;
  for (int i=n-1;i>=2;i--) args=cons(ctx,argv[i],args);

  // avoid gc
  pointer p=gensym(ctx);
  setval(ctx,intern(ctx,(char*)(p->c.sym.pname->c.str.chars),strlen((char*)(p->c.sym.pname->c.str.chars)),lisppkg),cons(ctx,fncallback,args));

  // ;; store mapTimered
  ROS_DEBUG("create timer %s at %f (oneshot=%d)", fncallname.c_str(), period, oneshot);
  s_mapTimered[fncallname] = lnode->createTimer(ros::Duration(period), TimerFunction(fncallback, args), oneshot);

  return (T);
}

/************************************************************
 *   __roseus
 ************************************************************/
extern pointer K_FUNCTION_DOCUMENTATION;
#include "defun.h"
pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env)
{
#ifdef ROSPACK_EXPORT
#else
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, 1);
#endif

  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("ROS", 3),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  QANON=defvar(ctx,"*ANONYMOUS-NAME*",makeint(ros::init_options::AnonymousName),rospkg);
  QNOOUT=defvar(ctx,"*NO-ROSOUT*",makeint(ros::init_options::NoRosout),rospkg);
  QROSDEBUG=defvar(ctx,"*ROSDEBUG*",makeint(1),rospkg);
  QROSINFO=defvar(ctx,"*ROSINFO*",makeint(2),rospkg);
  QROSWARN=defvar(ctx,"*ROSWARN*",makeint(3),rospkg);
  QROSERROR=defvar(ctx,"*ROSERROR*",makeint(4),rospkg);
  QROSFATAL=defvar(ctx,"*ROSFATAL*",makeint(5),rospkg);
  defun(ctx,"SPIN",argv[0],(pointer (*)())ROSEUS_SPIN, "Enter simple event loop");

  defun(ctx,"SPIN-ONCE",argv[0],(pointer (*)())ROSEUS_SPINONCE,
         "&optional groupname  ;; spin only group\n\n"
         "Process a single round of callbacks.\n");
  defun(ctx,"TIME-NOW-RAW",argv[0],(pointer (*)())ROSEUS_TIME_NOW, "");
  defun(ctx,"RATE",argv[0],(pointer (*)())ROSEUS_RATE, "frequency\n\n" "Construct ros timer for periodic sleeps");
  defun(ctx,"SLEEP",argv[0],(pointer (*)())ROSEUS_SLEEP, "Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.");
  defun(ctx,"DURATION-SLEEP",argv[0],(pointer (*)())ROSEUS_DURATION_SLEEP, "second\n\nSleeps for amount of the time specified by this duration.");
  defun(ctx,"OK",argv[0],(pointer (*)())ROSEUS_OK, "Check whether it's time to exit. ");

  defun(ctx,"ROS-DEBUG",argv[0],(pointer (*)())ROSEUS_ROSDEBUG,
         "write mesage to debug output\n"
         "\n"
         "	(ros::ros-debug \"this is error ~A\" 0)\n");
  defun(ctx,"ROS-INFO",argv[0],(pointer (*)())ROSEUS_ROSINFO, "write mesage to info output");
  defun(ctx,"ROS-WARN",argv[0],(pointer (*)())ROSEUS_ROSWARN, "write mesage to warn output");
  defun(ctx,"ROS-ERROR",argv[0],(pointer (*)())ROSEUS_ROSERROR, "write mesage to error output");
  defun(ctx,"ROS-FATAL",argv[0],(pointer (*)())ROSEUS_ROSFATAL, "write mesage to fatal output");
  defun(ctx,"EXIT",argv[0],(pointer (*)())ROSEUS_EXIT, "Exit ros clinet");

  defun(ctx,"SUBSCRIBE",argv[0],(pointer (*)())ROSEUS_SUBSCRIBE,
         "topicname message_type callbackfunc args0 ... argsN &optional (queuesize 1) %key (:groupname groupname)\n\n"
         "Subscribe to a topic, version for class member function with bare pointer.\n"
         "This method connects to the master to register interest in a given topic. The node will automatically be connected with publishers on this topic. On each message receipt, fp is invoked and passed a shared pointer to the message received. This message should not be changed in place, as it is shared with any other subscriptions to this topic.\n"
         "\n"
         "This version of subscribe is a convenience function for using function, member function, lmabda function:\n"
         "	;; callback function\n"
         "	(defun string-cb (msg) (print (list 'cb (sys::thread-self) (send msg :data))))\n"
         "	(ros::subscribe \"chatter\" std_msgs::string #'string-cb)\n"
         "	;; lambda function\n"
         "	(ros::subscribe \"chatter\" std_msgs::string\n"
         "	                #'(lambda (msg) (ros::ros-info\n"
         "	                                 (format nil \"I heard ~A\" (send msg :data)))))\n"
         "	;; method call\n"
         "	(defclass string-cb-class\n"
         "	  :super propertied-object\n"
         "	  :slots ())\n"
         "	(defmethod string-cb-class\n"
         "	  (:init () (ros::subscribe \"chatter\" std_msgs::string #'send self :string-cb))\n"
         "	  (:string-cb (msg) (print (list 'cb self (send msg :data)))))\n"
         "	(setq m (instance string-cb-class :init))\n"
         );
  defun(ctx,"UNSUBSCRIBE",argv[0],(pointer (*)())ROSEUS_UNSUBSCRIBE, "topicname\n\n""Unsubscribe topic");
  defun(ctx,"GET-NUM-PUBLISHERS",argv[0],(pointer (*)())ROSEUS_GETNUMPUBLISHERS, "Returns the number of publishers this subscriber is connected to. ");
  defun(ctx,"GET-TOPIC-SUBSCRIBER",argv[0],(pointer (*)())ROSEUS_GETTOPICSUBSCRIBER, "topicname\n\n""Retuns the name of topic if it already subscribed");
  defun(ctx,"ADVERTISE",argv[0],(pointer (*)())ROSEUS_ADVERTISE,
         "topic message_class &optional (queuesize 1) (latch nil)\n"
         "Advertise a topic.\n"
         "This call connects to the master to publicize that the node will be publishing messages on the given topic. This method returns a Publisher that allows you to publish a message on this topic.\n"
         "	(ros::advertise \"chatter\" std_msgs::string 1)");
  defun(ctx,"UNADVERTISE",argv[0],(pointer (*)())ROSEUS_UNADVERTISE, "Unadvertise topic");
  defun(ctx,"PUBLISH",argv[0],(pointer (*)())ROSEUS_PUBLISH,
         "topic message\n\n"
         "Publish a message on the topic\n"
         "	(ros::roseus \"talker\")\n"
         "	(ros::advertise \"chatter\" std_msgs::string 1)\n"
         "	(ros::rate 100)\n"
         "	(while (ros::ok)\n"
         "	  (setq msg (instance std_msgs::string :init))\n"
         "	  (send msg :data (format nil \"hello world ~a\" (send (ros::time-now) :sec-nsec)))\n"
         "	  (ros::publish \"chatter\" msg)\n"
         "	  (ros::sleep))\n");
  defun(ctx,"GET-NUM-SUBSCRIBERS",argv[0],(pointer (*)())ROSEUS_GETNUMSUBSCRIBERS, "Retuns number of subscribers this publish is connected to");
  defun(ctx,"GET-TOPIC-PUBLISHER",argv[0],(pointer (*)())ROSEUS_GETTOPICPUBLISHER, "topicname\n\n""Retuns the name of topic if it already published");

  defun(ctx,"WAIT-FOR-SERVICE",argv[0],(pointer (*)())ROSEUS_WAIT_FOR_SERVICE, "servicename &optional timeout\n\n""Wait for a service to be advertised and available. Blocks until it is.");
  defun(ctx,"SERVICE-EXISTS", argv[0], (pointer (*)())ROSEUS_SERVICE_EXISTS, "servicename\n\n""Checks if a service is both advertised and available.");
  defun(ctx,"SERVICE-CALL",argv[0],(pointer (*)())ROSEUS_SERVICE_CALL,
         "servicename message_type &optional persist\n\n"
         "Invoke RPC service\n"
         "	(ros::roseus \"add_two_ints_client\")\n"
         "	(ros::wait-for-service \"add_two_ints\")\n"
         "	(setq req (instance roseus::AddTwoIntsRequest :init))\n"
         "	(send req :a (random 10))\n"
         "	(send req :b (random 20))\n"
         "	(setq res (ros::service-call \"add_two_ints\" req t))\n"
         "	(format t \"~d + ~d = ~d~~%\" (send req :a) (send req :b) (send res :sum))\n");
  defun(ctx,"ADVERTISE-SERVICE",argv[0],(pointer (*)())ROSEUS_ADVERTISE_SERVICE,
         "servicename message_type callback function\n\n"
         "Advertise a service\n"
         "	(ros::advertise-service \"add_two_ints\" roseus::AddTwoInts #'add-two-ints)");
  defun(ctx,"UNADVERTISE-SERVICE",argv[0],(pointer (*)())ROSEUS_UNADVERTISE_SERVICE, "Unadvertise service");

  defun(ctx,"SET-PARAM",argv[0],(pointer (*)())ROSEUS_SET_PARAM, "key value\n\n""Set parameter");
  defun(ctx,"GET-PARAM",argv[0],(pointer (*)())ROSEUS_GET_PARAM, "key\n\n""Get parameter");
  defun(ctx,"GET-PARAM-CACHED",argv[0],(pointer (*)())ROSEUS_GET_PARAM_CACHED, "Get chached parameter");
  defun(ctx,"HAS-PARAM",argv[0],(pointer (*)())ROSEUS_HAS_PARAM, "Check whether a parameter exists on the parameter server.");
  defun(ctx,"DELETE-PARAM",argv[0],(pointer (*)())ROSEUS_DELETE_PARAM, "key\n\n""Delete parameter from server");
  defun(ctx,"SEARCH-PARAM",argv[0],(pointer (*)())ROSEUS_SEARCH_PARAM, "key\n\n""Search up the tree for a parameter with a given key. This version defaults to starting in the current node's name.");
  defun(ctx,"LIST-PARAM",argv[0],(pointer (*)())ROSEUS_LIST_PARAM, "Get the list of all the parameters in the server");

  defun(ctx,"ROSPACK-FIND",argv[0],(pointer (*)())ROSEUS_ROSPACK_FIND, "Returns ros package path");
  defun(ctx,"ROSPACK-PLUGINS",argv[0],(pointer (*)())ROSEUS_ROSPACK_PLUGINS, "Returns plugins of ros packages");
  defun(ctx,"ROSPACK-DEPENDS",argv[0],(pointer (*)())ROSEUS_ROSPACK_DEPENDS, "Returns ros package dependencies list");
  defun(ctx,"RESOLVE-NAME",argv[0],(pointer (*)())ROSEUS_RESOLVE_NAME, "Returns ros resolved name");
  defun(ctx,"GET-NAME",argv[0],(pointer (*)())ROSEUS_GETNAME, "Returns current node name");
  defun(ctx,"GET-NAMESPACE",argv[0],(pointer (*)())ROSEUS_GETNAMESPACE, "Returns current node name space");

  defun(ctx,"ROSEUS-RAW",argv[0],(pointer (*)())ROSEUS, "");
  defun(ctx,"CREATE-NODEHANDLE", argv[0], (pointer (*)())ROSEUS_CREATE_NODEHANDLE, "groupname &optional namespace  ;;\n\n"
         "Create ros NodeHandle with given group name. \n"
         "\n"
         "	(ros::roseus \"test\")\n"
         "	(ros::create-node-handle \"mygroup\")\n"
         "	(ros::subscribe \"/test\" std_msgs::String #'(lambda (m) (print m)) :groupname \"mygroup\")\n"
         "	(while (ros::ok)  (ros::spin-once \"mygroup\"))\n");
  defun(ctx,"SET-LOGGER-LEVEL",argv[0],(pointer (*)())ROSEUS_SET_LOGGER_LEVEL, "");

  defun(ctx,"GET-HOST",argv[0],(pointer (*)())ROSEUS_GET_HOST, "Get the hostname where the master runs.");
  defun(ctx,"GET-NODES",argv[0],(pointer (*)())ROSEUS_GET_NODES, "Retreives the currently-known list of nodes from the master.");
  defun(ctx,"GET-PORT",argv[0],(pointer (*)())ROSEUS_GET_PORT, "Get the port where the master runs.");
  defun(ctx,"GET-URI",argv[0],(pointer (*)())ROSEUS_GET_URI, "Get the full URI to the master ");
  defun(ctx,"GET-TOPICS",argv[0],(pointer (*)())ROSEUS_GET_TOPICS, "Get the list of topics that are being published by all nodes.");

  defun(ctx,"CREATE-TIMER",argv[0],(pointer (*)())ROSEUS_CREATE_TIMER, "Create periodic callbacks.");

  pointer_update(Spevalof(PACKAGE),p);

  pointer l;
  l=makestring(REPOVERSION,strlen(REPOVERSION));
  vpush(l);
  l=stacknlist(ctx,1);
  QREPOVERSION=defvar(ctx, "ROSEUS-REPO-VERSION", l, rospkg);

  M_string remappings;
  pointer argp = speval(intern(ctx,"*EUSTOP-ARGUMENT*", strlen("*EUSTOP-ARGUMENT*"),lisppkg));
  while(argp!=NIL) {
    std::string arg = string((char *)(ccar(argp)->c.str.chars));
    // copy from roscpp/src/init.cpp : 432
    size_t pos = arg.find(":=");
    if (pos != std::string::npos) {
      std::string local_name = arg.substr(0, pos);
      std::string external_name = arg.substr(pos + 2);
      remappings[local_name] = external_name;
    }
    argp=ccdr(argp);
  }
  ros::master::init(remappings);
  //ros::param::init(remappings);

  return 0;
}

