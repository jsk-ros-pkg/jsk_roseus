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

#include <ros/init.h>
#include <ros/time.h>
#include <ros/rate.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <rospack/rospack.h>
#include <ros/param.h>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string

#include "eus.h"
extern "C" {
  pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env);
  void register_roseus(){
    char modname[] = "___roseus";
    return add_module_initializer(modname, (pointer (*)())___roseus);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

extern int mainargc;
extern char* mainargv[32];

using namespace ros;
using namespace std;

class RoseusStaticData
{
public:
  RoseusStaticData() {}
  ~RoseusStaticData() {
  }
  boost::shared_ptr<ros::NodeHandle> node;
  ros::Rate *rate;
  map<string, boost::shared_ptr<Publisher> > mapAdvertised; ///< advertised topics
  map<string, boost::shared_ptr<Subscriber> > mapSubscribed; ///< subscribed topics
  map<string, boost::shared_ptr<ServiceServer> > mapServiced; ///< subscribed topics

};

static RoseusStaticData s_staticdata;
static bool s_bInstalled = false;
#define s_node s_staticdata.node
#define s_rate s_staticdata.rate
#define s_mapAdvertised s_staticdata.mapAdvertised
#define s_mapSubscribed s_staticdata.mapSubscribed
#define s_mapServiced s_staticdata.mapServiced

pointer K_ROSEUS_MD5SUM,K_ROSEUS_DATATYPE,K_ROSEUS_DEFINITION,K_ROSEUS_SERIALIZATION_LENGTH,K_ROSEUS_SERIALIZE,K_ROSEUS_DESERIALIZE,K_ROSEUS_INIT,K_ROSEUS_GET,K_ROSEUS_REQUEST,K_ROSEUS_RESPONSE,QANON,QNOOUT,QSVNVERSION;

/***********************************************************
 *   Message wrapper
 ************************************************************/

string getString(pointer message, pointer method) {
  context *ctx = current_ctx;
  pointer r, curclass;
  if ((pointer)findmethod(ctx,method,classof(message),&curclass)!=NIL) {
    ROS_ERROR("method");
    r = csend(ctx,message,method,0);
  } else if ((pointer)findmethod(ctx,K_ROSEUS_GET,classof(message),&curclass) != NIL ) {
    r = csend(ctx,message,K_ROSEUS_GET,1,method);
  } else {
    r = NULL;
#ifdef x86_64
    ROS_ERROR("could not find method %s for pointer %lx",
              get_string(method), message);
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
  a = (pointer)findmethod(ctx,method,classof(message),&curclass);
  if (a!=NIL) {
    vpush(message);
    pointer r = csend(ctx,message,method,0);
    vpop();
    return (ckintval(r));
  } else {
#ifdef x86_64
    ROS_ERROR("could not find method %s for pointer %lx",
              get_string(method), message);
#else
    ROS_ERROR("could not find method %s for pointer %x",
              get_string(method), (unsigned int)message);
#endif
  }
  return 0;
}

class EuslispMessage : public ros::Message
{
public:
  pointer _message;

  EuslispMessage(pointer message) : _message(message) {
  }
  EuslispMessage(const EuslispMessage &r) : Message() {
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

  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seqid) const
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
    memcpy(write_ptr, get_string(r), len);
    //ROS_INFO("serialize");

    return write_ptr + len;
  }

  virtual uint8_t *deserialize(uint8_t *readPtr)
  {
    context *ctx = current_ctx;
    if (ctx!=euscontexts[0])ROS_WARN("ctx is not correct %d\n",thr_self());
    pointer a,curclass;

    if ( __serialized_length == 0 ) {
      ROS_WARN("empty message!");
      return readPtr;
    }
    vpush(_message);
    a = (pointer)findmethod(ctx,K_ROSEUS_DESERIALIZE,classof(_message),&curclass);
    ROS_ASSERT(a!=NIL);
    pointer p = makestring((char *)readPtr,__serialized_length);
    pointer r = csend(ctx,_message,K_ROSEUS_DESERIALIZE,1,p);
    ROS_ASSERT(r!=NIL);
    //ROS_INFO("deserialize %d", __serialized_length);
    vpop(); // pop _message

    return readPtr+__serialized_length;
  }
};

/************************************************************
 *   Subscriptions
 ************************************************************/
#if ROS_VERSION_MINIMUM(1,1,0)
class EuslispSubscriptionCallbackHelper : public ros::SubscriptionCallbackHelper {
public:
  pointer _scb,_args;
  EuslispMessage _msg;

  EuslispSubscriptionCallbackHelper(pointer scb, pointer args,pointer tmpl) :  _args(args), _msg(tmpl) {
    extern pointer LAMCLOSURE;
    context *ctx = current_ctx;
    //ROS_WARN("func");prinx(ctx,scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,args,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    if (piscode(scb)) {
      _scb=scb;
    } else if ((ccar(scb))==LAMCLOSURE) {
      if ( ccar(ccdr(scb)) != NIL ) {
        _scb=ccar(ccdr(scb));
      } else { // defvar lambda function
        _scb=gensym(ctx);
        pointer a = DEFUN(ctx,(cons(ctx,_scb,(ccdr(ccdr(ccdr(ccdr(scb))))))));
        defvar(ctx,(char *)(get_string(_scb)),a,Spevalof(PACKAGE));
      }
    } else {
      ROS_ERROR("subcriptoin callback function install error");
    }
    // avoid gc
    deflocal(ctx,(char *)get_string(gensym(ctx)),_args,Spevalof(PACKAGE));
  }
  ~EuslispSubscriptionCallbackHelper() {
  }
  virtual ros::VoidConstPtr deserialize(const ros::SubscriptionCallbackHelperDeserializeParams& param) {
#if DEBUG
    cerr << __PRETTY_FUNCTION__ << endl;
    cerr << "param.length = " << param.length << endl;
    cerr << "param.buffer = " << (param.buffer + 4) << endl;
    cerr << "c_header == " << param.connection_header << endl;
#endif
    ros::VoidConstPtr ptr(new EuslispMessage(_msg));
    EuslispMessage *eus_msg = (EuslispMessage *)(ptr.get());
    (*eus_msg).__serialized_length = param.length;
    eus_msg->deserialize(param.buffer);

    return ptr;
  }
  virtual void call(ros::SubscriptionCallbackHelperCallParams& param) {
    EuslispMessage* eus_msg = (EuslispMessage *)((void *)param.event.getConstMessage().get());
    context *ctx = current_ctx;
    pointer func=_scb,argp=_args;
    int argc=0;
    //ROS_WARN("func");prinx(ctx,_scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,argp,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    vpush(eus_msg->_message);    // to avoid GC
    if ( issymbol(_scb) ) {
      func = FUNCTION_CLOSURE(ctx,(cons(ctx,_scb,NIL)));
    } else if ( piscode(func) ) {
      //
    } else {
      ROS_ERROR("cant't find callback function");
    }
    vpush(func);
    
    while(argp!=NIL){ ckpush(ccar(argp)); argp=ccdr(argp); argc++;}
    vpush((pointer)(eus_msg->_message));argc++;
    
    ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),func,(pointer)(ctx->vsp-argc),NULL,argc);
    while(argc-->0)vpop();
    vpop();                     // remove function
    vpop();    // remove eus_msg._message
  }
  virtual const std::type_info& getTypeInfo() {
    return typeid(EuslispMessage);
  }
  virtual bool isConst(){
    return true;
  }
};
#else
class EuslispSubscriptionMessageHelper : public ros::SubscriptionMessageHelper {
public:
  pointer _scb,_args;
  EuslispMessage _msg;
  string md5, datatype, defnition;

  EuslispSubscriptionMessageHelper(pointer scb, pointer args,pointer tmpl) :  _args(args), _msg(tmpl) {
    extern pointer LAMCLOSURE;
    context *ctx = current_ctx;

    //ROS_WARN("func");prinx(ctx,scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,args,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    if (piscode(scb)) {
      _scb=scb;
    } else if ((ccar(scb))==LAMCLOSURE) {
      if ( ccar(ccdr(scb)) != NIL ) {
        _scb=ccar(ccdr(scb));
      } else { // defvar lambda function
        _scb=gensym(ctx);
        pointer a = DEFUN(ctx,(cons(ctx,_scb,(ccdr(ccdr(ccdr(ccdr(scb))))))));
        defvar(ctx,(char *)(get_string(_scb)),a,Spevalof(PACKAGE));
      }
    } else {
      ROS_ERROR("subcriptoin callback function install error");
    }
    // avoid gc
    deflocal(ctx,(char *)get_string(gensym(ctx)),_args,Spevalof(PACKAGE));
    md5 = _msg.__getMD5Sum();
    datatype = _msg.__getDataType();
    definition = _msg.__getMessageDefinition();
  }
  ~EuslispSubscriptionMessageHelper() {}

  virtual MessagePtr create() { return boost::shared_ptr<Message>(new EuslispMessage(_msg)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }
  virtual std::string getMessageDefinition() { return definition; }

  virtual void call(const MessagePtr &msg) {
    context *ctx = current_ctx;
    pointer func=_scb,argp=_args;
    int argc=0;
    //ROS_WARN("func");prinx(ctx,_scb,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    //ROS_WARN("argc");prinx(ctx,argp,ERROUT);flushstream(ERROUT);terpri(ERROUT);
    while(argp!=NIL){ ckpush(ccar(argp)); argp=ccdr(argp); argc++;}
    vpush((pointer)((EuslispMessage *)msg.get())->_message);argc++;
    if ( issymbol(_scb) ) {
      func = FUNCTION_CLOSURE(ctx,(cons(ctx,_scb,NIL)));
    } else if ( piscode(func) ) {
      //
    } else {
      ROS_ERROR("cant't find callback function");
    }

    ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),func,(pointer)(ctx->vsp-argc),NULL,argc);
    while(argc-->0)vpop();
  }
};
#endif

/************************************************************
 *   ServiceCall
 ************************************************************/
#if ROS_VERSION_MINIMUM(1,1,0)
class EuslispServiceCallbackHelper : public ros::ServiceCallbackHelper {
public:
  pointer _scb;
  EuslispMessage _req, _res;
  string md5, datatype, requestDataType, responseDataType,
    requestMessageDefinition, responseMessageDefinition;

  EuslispServiceCallbackHelper(pointer scb, string smd5, string sdatatype, pointer reqclass, pointer resclass) : _req(reqclass), _res(resclass), md5(smd5), datatype(sdatatype) {
    extern pointer LAMCLOSURE;
    context *ctx = current_ctx;
    if ((ccar(scb))==LAMCLOSURE) {
      if ( ccar(ccdr(scb)) != NIL ) {
        _scb=ccar(ccdr(scb));
      } else { // defvar lambda function
        _scb=gensym(ctx);
        vpush(_scb);
        vpush(scb);
        pointer a = DEFUN(ctx,(cons(ctx,_scb,(ccdr(ccdr(ccdr(ccdr(scb))))))));
        defvar(ctx,(char *)(get_string(_scb)),a,Spevalof(PACKAGE));
        vpop();
        vpop();
      }
    } else  {
      ROS_ERROR("service callback function install error");
    }
    requestDataType = _req.__getDataType();
    responseDataType = _res.__getDataType();
    requestMessageDefinition = _req.__getMessageDefinition();
    responseMessageDefinition = _res.__getMessageDefinition();
  }
  ~EuslispServiceCallbackHelper() { }

  virtual MessagePtr createRequest() { return boost::shared_ptr<Message>(new EuslispMessage(_req)); }
  virtual MessagePtr createResponse() { return boost::shared_ptr<Message>(new EuslispMessage(_res)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }
  virtual std::string getRequestDataType() { return requestDataType; }
  virtual std::string getResponseDataType() { return responseDataType; }
  virtual std::string getRequestMessageDefinition() { return requestMessageDefinition; }
  virtual std::string getResponseMessageDefinition() { return responseMessageDefinition; }

  virtual bool call(ros::ServiceCallbackHelperCallParams& params) {
    context *ctx = current_ctx;
    pointer func = _scb, r;
    vpush(_res._message);       // _res._message
    vpush(_req._message);       // _res._message, _req._message
    if ( issymbol(_scb) ) {
      func = FUNCTION_CLOSURE(ctx,(cons(ctx,_scb,NIL)));
    } else {
      ROS_ERROR("cant't find callback function");
    }
    vpush(func);                // _res._message, _req._message, func
    // Deserialization
    EuslispMessage eus_msg(_req);
    eus_msg.__serialized_length = params.request.num_bytes;
    vpush(eus_msg._message);    // _res._message, _req._message, func, eus_msg._message
    eus_msg.deserialize(params.request.message_start);
    r = ufuncall(ctx, (ctx->callfp?ctx->callfp->form:NIL),
                 func, (pointer)(ctx->vsp-1),
                 NULL, 1);
    vpush(r); // _res._message, _req._message, func, eus_msg._message, r, eus_res._message
    // Serializaion
    EuslispMessage eus_res(_res);
    eus_res.replaceContents(r);
    vpush(eus_res._message);    // _res._message, _req._message, func, eus_msg._message, r, eus_res._message
    
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
#if DEBUG
    cerr << "num bytes = " << params.response.num_bytes << endl;
    ROS_INFO("message_start =  %X",params.response.message_start);
    ROS_INFO("message_ptr =  %X",params.response.buf.get());
    tmp = params.response.buf.get();
    for(int i=0;i<params.response.num_bytes;i++){
      ROS_INFO("%X", tmp[i]);
    }
#endif
    vpop(); // _res._message, _req._message, func, eus_msg._message, r, eus_res._message
    vpop(); // _res._message, _req._message, func, eus_msg._message, r
    vpop(); // _res._message, _req._message, func, eus_msg._message
    vpop(); // _res._message, _req._message, func,
    vpop(); // _res._message, _req._message,
    vpop(); // _res._message
    return(T);
  }
};
#else
class EuslispServiceMessageHelper : public ros::ServiceMessageHelper {
public:
  pointer _scb;
  EuslispMessage _req, _res;
  string md5, datatype, requestDataType, responseDataType, requestMessageDefinition, responseMessageDefinition;

  EuslispServiceMessageHelper(pointer scb, string smd5, string sdatatype, pointer reqclass, pointer resclass) : _req(reqclass), _res(resclass), md5(smd5), datatype(sdatatype) {
    extern pointer LAMCLOSURE;
    context *ctx = current_ctx;
    if ((ccar(scb))==LAMCLOSURE) {
      if ( ccar(ccdr(scb)) != NIL ) {
        _scb=ccar(ccdr(scb));
      } else { // defvar lambda function
        _scb=gensym(ctx);
        pointer a = DEFUN(ctx,(cons(ctx,_scb,(ccdr(ccdr(ccdr(ccdr(scb))))))));
        defvar(ctx,(char *)(get_string(_scb)),a,Spevalof(PACKAGE));
      }
    } else  {
      ROS_ERROR("service callback function install error");
    }
    requestDataType = _req.__getDataType();
    responseDataType = _res.__getDataType();
    requestMessageDefinition = _req.__getMessageDefinition();
    responseMessageDefinition = _res.__getMessageDefinition();
  }
  ~EuslispServiceMessageHelper() { }

  virtual MessagePtr createRequest() { return boost::shared_ptr<Message>(new EuslispMessage(_req)); }
  virtual MessagePtr createResponse() { return boost::shared_ptr<Message>(new EuslispMessage(_res)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }
  virtual std::string getRequestDataType() { return requestDataType; }
  virtual std::string getResponseDataType() { return responseDataType; }
  virtual std::string getRequestMessageDefinition() { return requestMessageDefinition; }
  virtual std::string getResponseMessageDefinition() { return responseMessageDefinition; }

  virtual bool call(const MessagePtr &req, const MessagePtr &res) {
    context *ctx = current_ctx;
    pointer func = _scb, r;

    vpush((pointer)((EuslispMessage *)req.get())->_message);
    if ( issymbol(_scb) ) {
      func = FUNCTION_CLOSURE(ctx,(cons(ctx,_scb,NIL)));
    } else {
      ROS_ERROR("cant't find callback function");
    }

    r = ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),func,(pointer)(ctx->vsp-1),NULL,1);
    vpop();

    ((EuslispMessage *)res.get())->replaceContents(r);
    return(T);
  }
};
#endif

void roseusSignalHandler(int sig)
{
    // firs of all, call ros signal handller
    ros::requestShutdown();
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

  for (unsigned int i=0; i < strlen(name); i++)
    if ( ! isalpha(name[i]) ) name[i] = '_';

  K_ROSEUS_MD5SUM   = defkeyword(ctx,"MD5SUM-");
  K_ROSEUS_DATATYPE = defkeyword(ctx,"DATATYPE-");
  K_ROSEUS_DEFINITION = defkeyword(ctx,"DEFINITION-");
  K_ROSEUS_SERIALIZATION_LENGTH = defkeyword(ctx,"SERIALIZATION-LENGTH");
  K_ROSEUS_SERIALIZE   = defkeyword(ctx,"SERIALIZE");
  K_ROSEUS_DESERIALIZE = defkeyword(ctx,"DESERIALIZE");
  K_ROSEUS_GET  = defkeyword(ctx,"GET");
  K_ROSEUS_INIT = defkeyword(ctx,"INIT");
  K_ROSEUS_REQUEST  = defkeyword(ctx,"REQUEST");
  K_ROSEUS_RESPONSE = defkeyword(ctx,"RESPONSE");

  s_mapAdvertised.clear();
  s_mapSubscribed.clear();
  s_mapServiced.clear();
  
  /*
    force to flag ros::init_options::NoSigintHandler.
    In fact, this code make no sence, because we steals
    SIGINT handler by the following `signal'.
   */
  options |= ros::init_options::NoSigintHandler;
  
  ros::init(cargc, cargv, name, options);

  s_node.reset(new ros::NodeHandle());

  s_bInstalled = true;

  // install signal handler for sigint. DO NOT call unix:signal after
  // ros::roseus
  signal(SIGINT, roseusSignalHandler);
  return (T);
}

pointer ROSEUS_SPIN(register context *ctx,int n,pointer *argv)
{
  ros::spin();
  return (NIL);
}

pointer ROSEUS_SPINONCE(register context *ctx,int n,pointer *argv)
{
  ros::spinOnce();
  return (NIL);
}

pointer ROSEUS_TIME_NOW(register context *ctx,int n,pointer *argv)
{
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
  numunion nu;
  ckarg(1);
  float timeout=ckfltval(argv[0]);
  s_rate = new ros::Rate(timeout);
  return(T);
}

pointer ROSEUS_SLEEP(register context *ctx,int n,pointer *argv)
{
  s_rate->sleep();
  return (T);
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
    ROS_INFO("exiting roseus");
    s_mapAdvertised.clear();
    s_mapSubscribed.clear();
    s_mapServiced.clear();
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
  string topicname;
  pointer message, fncallback, args;
  int queuesize = 1;

  if (isint(argv[n-1])) {queuesize = ckintval(argv[n-1]);n--;}
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  // TODO:need error checking
  message = argv[1];
  fncallback = argv[2];
  args=NIL;
  for (int i=n-1;i>=3;i--) args=cons(ctx,argv[i],args);

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  EuslispMessage msg(message);
#if ROS_VERSION_MINIMUM(1,1,0)
   boost::shared_ptr<SubscriptionCallbackHelper> *callback =
     (new boost::shared_ptr<SubscriptionCallbackHelper>
      (new EuslispSubscriptionCallbackHelper(fncallback, args, message)));
  SubscribeOptions so(topicname, queuesize, msg.__getMD5Sum(), msg.__getDataType());
  so.helper = *callback;
#else
  boost::shared_ptr<SubscriptionMessageHelper> *callback = (new boost::shared_ptr<SubscriptionMessageHelper>(new EuslispSubscriptionMessageHelper(fncallback,args,message)));
  SubscribeOptions so(topicname,queuesize,*callback);
#endif
  Subscriber subscriber = s_node->subscribe(so);
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
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  map<string, boost::shared_ptr<Subscriber> >::iterator it = s_mapSubscribed.find(topicname);

  bool bSuccess = s_mapSubscribed.erase(topicname)>0;

  return (bSuccess?T:NIL);
}

pointer ROSEUS_GETNUMPUBLISHERS(register context *ctx,int n,pointer *argv)
{
  string topicname;
  int ret;

  ckarg(1);
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Subscriber> >::iterator it = s_mapSubscribed.find(topicname);
  if( it != s_mapSubscribed.end() ) {
    boost::shared_ptr<Subscriber> subscriber = (it->second);
    ret = subscriber->getNumPublishers();
    bSuccess = true;
  }

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to getNumPublishers to topic %s, which was not " \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (bSuccess?(makeint(ret)):NIL);
}

pointer ROSEUS_GETTOPICSUBSCRIBER(register context *ctx,int n,pointer *argv)
{
  string topicname;
  string ret;

  ckarg(1);
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Subscriber> >::iterator it = s_mapSubscribed.find(topicname);
  if( it != s_mapSubscribed.end() ) {
    boost::shared_ptr<Subscriber> subscriber = (it->second);
    ret = subscriber->getTopic();
    bSuccess = true;
  }

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to getTopic to topic %s, which was not " \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (bSuccess?(makestring((char *)ret.c_str(), ret.length())):NIL);
}

pointer ROSEUS_ADVERTISE(register context *ctx,int n,pointer *argv)
{
  string topicname;
  pointer message;
  int queuesize = 1;

  ckarg2(2,3);
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  message = argv[1];
  if ( n > 2 ) {
    queuesize = ckintval(argv[2]);
  }

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  if( s_mapAdvertised.find(topicname) != s_mapAdvertised.end() ) {
    ROS_INFO("topic already advertised");
    return (NIL);
  }

  EuslispMessage msg(message);
  AdvertiseOptions ao(topicname, queuesize, msg.__getMD5Sum(), msg.__getDataType(), msg.__getMessageDefinition());
  //ao.latch = latch;
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
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  map<string, boost::shared_ptr<Publisher> >::iterator it = s_mapAdvertised.find(topicname);

  bool bSuccess = s_mapAdvertised.erase(topicname)>0;

  return (bSuccess?T:NIL);
}

pointer ROSEUS_PUBLISH(register context *ctx,int n,pointer *argv)
{
  string topicname;
  pointer emessage;

  ckarg(2);
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  emessage = argv[1];

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

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
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
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
  if (isstring(argv[0])) topicname.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = false;
  map<string, boost::shared_ptr<Publisher> >::iterator it = s_mapAdvertised.find(topicname);
  if( it != s_mapAdvertised.end() ) {
    boost::shared_ptr<Publisher> publisher = (it->second);
    ret = publisher->getTopic();
    bSuccess = true;
  }

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to getTopic to topic %s, which was not " \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (bSuccess?(makestring((char *)ret.c_str(), ret.length())):NIL);
}

/************************************************************
 *   ROSEUS ServiceCall
 ************************************************************/
pointer ROSEUS_WAIT_FOR_SERVICE(register context *ctx,int n,pointer *argv)
{
  string service;

  ckarg2(1,2);
  if (isstring(argv[0])) service.assign((char *)(argv[0]->c.str.chars));
  else error(E_NOSTRING);

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  int32_t timeout = -1;

  if( n > 1 )
    timeout = (int32_t)ckintval(argv[1]);

  bool bSuccess = service::waitForService(ros::names::resolve(service), timeout);

  return (bSuccess?T:NIL);
}

pointer ROSEUS_SERVICE_CALL(register context *ctx,int n,pointer *argv)
{
  string service;
  pointer emessage;
  bool persist = false;

  ckarg(2);
  if (isstring(argv[0])) service.assign((char *)(argv[0]->c.str.chars));
  else error(E_NOSTRING);
  emessage = argv[1];

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  EuslispMessage request(emessage);
  vpush(request._message);      // to avoid GC, it may not be required...
  EuslispMessage response(csend(ctx,emessage,K_ROSEUS_RESPONSE,0));
  vpush(response._message);     // to avoid GC, its important
  ServiceClientOptions sco(ros::names::resolve(service), request.__getMD5Sum(), persist, M_string());
  ServiceClient client = s_node->serviceClient(sco);
  ServiceClient* srv = new ServiceClient(client);
#if !ROS_VERSION_MINIMUM(1,1,0)
  // service client call is called from different thread, in boxturtle
  // this confuses euslisp ctx, we assume caller thread is 64
  for (int i=0;i<1;i++) euscontexts[MAXTHREAD+i] = euscontexts[0];
#endif
  // NEED FIX
  bool bSuccess =  srv->call(request, response, request.__getMD5Sum());
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
  string service;
  pointer emessage;
  pointer fncallback;

  ckarg(3);
  if (isstring(argv[0])) service.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  emessage = argv[1];
  fncallback = argv[2];

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  EuslispMessage message(emessage);
  vpush(message._message);      // to avoid GC in csend
  pointer request(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_REQUEST));
  pointer response(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_RESPONSE));
  vpop();                       // pop message._message
#if ROS_VERSION_MINIMUM(1,1,0)
  boost::shared_ptr<EuslispServiceCallbackHelper> *callback =
    (new boost::shared_ptr<EuslispServiceCallbackHelper>
     (new EuslispServiceCallbackHelper(fncallback, message.__getMD5Sum(),
                                       message.__getDataType(), request, response)));
  AdvertiseServiceOptions aso;
  aso.service.assign(service);
  aso.datatype = (*callback->get()).getDataType();
  aso.md5sum = (*callback->get()).getMD5Sum();
  aso.req_datatype = (*callback->get()).getRequestDataType();
  aso.res_datatype = (*callback->get()).getResponseDataType();
  aso.helper = *callback;
#else
  boost::shared_ptr<EuslispServiceMessageHelper> *callback = (new boost::shared_ptr<EuslispServiceMessageHelper>(new EuslispServiceMessageHelper(fncallback, message.__getMD5Sum(), message.__getDataType(), request, response)));
  AdvertiseServiceOptions aso(service, *callback);
#endif
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
  if (isstring(argv[0])) service.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  map<string, boost::shared_ptr<ServiceServer> >::iterator it = s_mapServiced.find(service);

  ROS_DEBUG("unadvertise %s", service.c_str());
  bool bSuccess = s_mapServiced.erase(service)>0;

  return (bSuccess?T:NIL);
}

pointer ROSEUS_SET_PARAM(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;
  string s;
  double d;
  int i;

  ckarg(2);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  if ( isstring(argv[1]) ) {
    s.assign((char *)get_string(argv[1]));
    ros::param::set(key,s);
  } else if (isint(argv[1])) {
    i = intval(argv[1]);
    ros::param::set(key,i);
  } else if (isflt(argv[1])) {
    d = fltval(argv[1]);
    ros::param::set(key,d);
  } else {
    error(E_MISMATCHARG);
  }
  return (T);
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
            else {
                ROS_FATAL("unkown rosparam type!");
                vpop();         // remove vpush(ret)
                return NIL;
            }
        }
        vpop();                 // remove vpush(ret)
        return ccdr(first);
    } else
        return (NIL);
}

pointer ROSEUS_GET_PARAM(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;

  ckarg(1);
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
    ROS_ERROR("unknown getParam type");
    return (NIL);
  }
  return (ret);
}

pointer ROSEUS_GET_PARAM_CASHED(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;

  ckarg(1);
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
    ROS_ERROR("unknown getParam type");
    return (NIL);
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

pointer ROSEUS_ROSPACK_FIND(register context *ctx,int n,pointer *argv)
{
  ckarg(1);

  string pkg;
  if (isstring(argv[0])) pkg.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  rospack::ROSPack rp;
  try {
    rospack::Package *p = rp.get_pkg(pkg);
    if (p!=NULL) return(makestring((char *)p->path.c_str(),p->path.length()));
  } catch (runtime_error &e) {
  }
  return(NIL);
}

pointer ROSEUS_GETNAME(register context *ctx,int n,pointer *argv)
{
  ckarg(0);
  return(makestring((char *)ros::this_node::getName().c_str(),
		    ros::this_node::getName().length()));
}

pointer ROSEUS_IS_INITIALIZED(register context *ctx,int n,pointer *argv)
{
  ckarg(0);
  return(s_bInstalled?T:NIL);
}

/************************************************************
 *   __roseus
 ************************************************************/

namespace ros {
  namespace master {
    void init(const M_string& remappings);
  }
  namespace param {
    void init(const M_string& remappings);
  }
}

pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("ROS", 3),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  QANON=defvar(ctx,"*ANONYMOUS-NAME*",makeint(ros::init_options::AnonymousName),rospkg);
  QNOOUT=defvar(ctx,"*NO-ROSOUT*",makeint(ros::init_options::NoRosout),rospkg);
  defun(ctx,"SPIN",argv[0],(pointer (*)())ROSEUS_SPIN);
  defun(ctx,"SPIN-ONCE",argv[0],(pointer (*)())ROSEUS_SPINONCE);
  defun(ctx,"TIME-NOW-RAW",argv[0],(pointer (*)())ROSEUS_TIME_NOW);
  defun(ctx,"RATE",argv[0],(pointer (*)())ROSEUS_RATE);
  defun(ctx,"SLEEP",argv[0],(pointer (*)())ROSEUS_SLEEP);
  defun(ctx,"OK",argv[0],(pointer (*)())ROSEUS_OK);

  defun(ctx,"ROS-DEBUG",argv[0],(pointer (*)())ROSEUS_ROSDEBUG);
  defun(ctx,"ROS-INFO",argv[0],(pointer (*)())ROSEUS_ROSINFO);
  defun(ctx,"ROS-WARN",argv[0],(pointer (*)())ROSEUS_ROSWARN);
  defun(ctx,"ROS-ERROR",argv[0],(pointer (*)())ROSEUS_ROSERROR);
  defun(ctx,"ROS-FATAL",argv[0],(pointer (*)())ROSEUS_ROSFATAL);
  defun(ctx,"EXIT",argv[0],(pointer (*)())ROSEUS_EXIT);

  defun(ctx,"SUBSCRIBE",argv[0],(pointer (*)())ROSEUS_SUBSCRIBE);
  defun(ctx,"UNSUBSCRIBE",argv[0],(pointer (*)())ROSEUS_UNSUBSCRIBE);
  defun(ctx,"GET-NUM-PUBLISHERS",argv[0],(pointer (*)())ROSEUS_GETNUMPUBLISHERS);
  defun(ctx,"GET-TOPIC-SUBSCRIBER",argv[0],(pointer (*)())ROSEUS_GETTOPICSUBSCRIBER);

  defun(ctx,"ADVERTISE",argv[0],(pointer (*)())ROSEUS_ADVERTISE);
  defun(ctx,"UNADVERTISE",argv[0],(pointer (*)())ROSEUS_UNADVERTISE);
  defun(ctx,"PUBLISH",argv[0],(pointer (*)())ROSEUS_PUBLISH);
  defun(ctx,"GET-NUM-SUBSCRIBERS",argv[0],(pointer (*)())ROSEUS_GETNUMSUBSCRIBERS);
  defun(ctx,"GET-TOPIC-PUBLISHER",argv[0],(pointer (*)())ROSEUS_GETTOPICPUBLISHER);

  defun(ctx,"WAIT-FOR-SERVICE",argv[0],(pointer (*)())ROSEUS_WAIT_FOR_SERVICE);
  defun(ctx,"SERVICE-CALL",argv[0],(pointer (*)())ROSEUS_SERVICE_CALL);
  defun(ctx,"ADVERTISE-SERVICE",argv[0],(pointer (*)())ROSEUS_ADVERTISE_SERVICE);
  defun(ctx,"UNADVERTISE-SERVICE",argv[0],(pointer (*)())ROSEUS_UNADVERTISE_SERVICE);

  defun(ctx,"SET-PARAM",argv[0],(pointer (*)())ROSEUS_SET_PARAM);
  defun(ctx,"GET-PARAM",argv[0],(pointer (*)())ROSEUS_GET_PARAM);
  defun(ctx,"GET-PARAM-CASHED",argv[0],(pointer (*)())ROSEUS_GET_PARAM_CASHED);
  defun(ctx,"HAS-PARAM",argv[0],(pointer (*)())ROSEUS_HAS_PARAM);

  defun(ctx,"ROSPACK-FIND",argv[0],(pointer (*)())ROSEUS_ROSPACK_FIND);
  defun(ctx,"GET-NAME",argv[0],(pointer (*)())ROSEUS_GETNAME);
  defun(ctx,"IS-INITIALIZED",argv[0],(pointer (*)())ROSEUS_IS_INITIALIZED);

  defun(ctx,"ROSEUS-RAW",argv[0],(pointer (*)())ROSEUS);

  pointer_update(Spevalof(PACKAGE),p);

  pointer l;
  l=makestring(SVNVERSION,strlen(SVNVERSION));
  vpush(l);
  l=stacknlist(ctx,1);
  QSVNVERSION=defvar(ctx, "ROSEUS-SVNVERSION", l, rospkg);

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

