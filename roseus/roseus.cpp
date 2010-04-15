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

pointer K_ROSEUS_MD5SUM,K_ROSEUS_TYPE,K_ROSEUS_SERIALIZATION_LENGTH,K_ROSEUS_SERIALIZE,K_ROSEUS_DESERIALIZE,K_ROSEUS_INIT,K_ROSEUS_GET,K_ROSEUS_REQUEST,K_ROSEUS_RESPONSE,QANON,QNOINT,QNOOUT;

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
    ROS_ERROR("could not find method %s for pointer %x",
              get_string(method), (unsigned int)message);
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
    pointer r = csend(ctx,message,method,0);
    return (ckintval(r));
  } else {
    ROS_ERROR("could not find method %s for pointer %x",
              get_string(method), (unsigned int)message);
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
      _message = makeobject(r._message);
      csend(ctx,_message,K_ROSEUS_INIT,0);
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
    return getString(_message, K_ROSEUS_TYPE);
  }
  virtual const string __getMD5Sum()   const {
    return getString(_message, K_ROSEUS_MD5SUM);
  }
  virtual const string __getMessageDefinition() const { return ""; }
  virtual const string __getServiceDataType() const {
    return getString(_message, K_ROSEUS_TYPE);
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
    uint32_t len = serializationLength();

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
    a = (pointer)findmethod(ctx,K_ROSEUS_DESERIALIZE,classof(_message),&curclass);
    ROS_ASSERT(a!=NIL);
    pointer p = makestring((char *)readPtr,__serialized_length);
    pointer r = csend(ctx,_message,K_ROSEUS_DESERIALIZE,1,p);
    ROS_ASSERT(r!=NIL);
    //ROS_INFO("deserialize %d", __serialized_length);

    return readPtr+__serialized_length;
  }
};

/************************************************************
 *   Subscriptions
 ************************************************************/
class EuslispSubscriptionMessageHelper : public ros::SubscriptionMessageHelper {
public:
  pointer _scb,_args;
  EuslispMessage _msg;
  string md5, datatype;

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
  }
  ~EuslispSubscriptionMessageHelper() {}

  virtual MessagePtr create() { return boost::shared_ptr<Message>(new EuslispMessage(_msg)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }

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

/************************************************************
 *   ServiceCall
 ************************************************************/

class EuslispServiceMessageHelper : public ros::ServiceMessageHelper {
public:
  pointer _scb;
  EuslispMessage _req, _res;
  string md5, datatype, requestDataType, responseDataType;

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
  }
  ~EuslispServiceMessageHelper() { }

  virtual MessagePtr createRequest() { return boost::shared_ptr<Message>(new EuslispMessage(_req)); }
  virtual MessagePtr createResponse() { return boost::shared_ptr<Message>(new EuslispMessage(_res)); }

  virtual std::string getMD5Sum() { return md5; }
  virtual std::string getDataType() { return datatype; }
  virtual std::string getRequestDataType() { return requestDataType; }
  virtual std::string getResponseDataType() { return responseDataType; }

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
/************************************************************
 *   EUSLISP functions
 ************************************************************/
pointer ROSEUS(register context *ctx,int n,pointer *argv)
{
  char name[256] = "";
  uint32_t options = 0;
  int cargc = 0;
  char *cargv[32];

  if( s_bInstalled ) return (T);

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

  K_ROSEUS_MD5SUM = defkeyword(ctx,"MD5SUM");
  K_ROSEUS_TYPE   = defkeyword(ctx,"TYPE");
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

  ros::init(cargc, cargv, name, options);

  s_node.reset(new ros::NodeHandle());

  s_bInstalled = true;

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
  {                                                             \
    char *msg = "";                                             \
    ckarg2(0,1);                                                \
    if (n == 1 ) {                                              \
      if (isstring(argv[0])) {                                  \
        msg = (char *)(argv[0]->c.str.chars);                   \
      } else { /* prin1-to-string */                            \
        extern pointer PRSTRUCTURE;                             \
        pointer p, s;                                           \
        { /* make-string-stream */                              \
          s=makeobject(C_STREAM);                               \
          csend(ctx,s,K_ROSEUS_INIT,2,K_OUT,makeint(256));      \
        }                                                       \
        { /* prin1 */                                           \
          p=Spevalof(PRSTRUCTURE);                              \
          Spevalof(PRSTRUCTURE)=T;                              \
          prinx(ctx,argv[0],s);                                 \
          Spevalof(PRSTRUCTURE)=p;                              \
        }                                                       \
        { /* get-output-stream-string */                        \
          char buf[256];                                        \
          int len = intval(s->c.obj.iv[3]);                     \
          if ( len >= 255 ) len = 255;                          \
          memcpy(&buf,s->c.obj.iv[2]->c.str.chars,len);         \
          buf[len]=0;                                           \
          msg = (char *)&buf;                                   \
        }                                                       \
      }                                                         \
    }                                                           \
    rosfuncname("%s", msg);                                     \
    return (T);                                                 \
  }

def_rosconsole_formatter(ROSEUS_ROSWARN,  ROS_WARN)
def_rosconsole_formatter(ROSEUS_ROSERROR, ROS_ERROR)
def_rosconsole_formatter(ROSEUS_ROSFATAL, ROS_FATAL)
def_rosconsole_formatter(ROSEUS_ROSINFO,  ROS_INFO)

pointer ROSEUS_EXIT(register context *ctx,int n,pointer *argv)
{
  ROS_INFO("%s", __PRETTY_FUNCTION__);
  if( s_bInstalled ) {
    ROS_INFO("exiting roseus");
    s_mapAdvertised.clear();
    s_mapSubscribed.clear();
    s_mapServiced.clear();
    ros::shutdown();
    return (NIL);
  }
  return (T);
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
  boost::shared_ptr<SubscriptionMessageHelper> *callback = (new boost::shared_ptr<SubscriptionMessageHelper>(new EuslispSubscriptionMessageHelper(fncallback,args,message)));
  SubscribeOptions so(topicname,queuesize,*callback);
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

  bool bSuccess = service::waitForService(service, timeout);

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
  EuslispMessage response(csend(ctx,emessage,K_ROSEUS_RESPONSE,0));
  ServiceClientOptions sco(service, request.__getMD5Sum(), persist, M_string());
  ServiceClient client = s_node->serviceClient(sco);
  ServiceClient* srv = new ServiceClient(client);
  // NEED FIX
  // service client call is called from different thread,
  // this confuses euslisp ctx, we assume caller thread is 64
  for (int i=0;i<1;i++) euscontexts[MAXTHREAD+i] = euscontexts[0];
  // NEED FIX
  bool bSuccess =  srv->call(request, response, request.__getMD5Sum());

  if ( ! bSuccess ) {
    ROS_ERROR("attempted to cass service  %s, but failed ",
              service.c_str());
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
  pointer request(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_REQUEST));
  pointer response(csend(ctx,emessage,K_ROSEUS_GET,1,K_ROSEUS_RESPONSE));
  boost::shared_ptr<EuslispServiceMessageHelper> *callback = (new boost::shared_ptr<EuslispServiceMessageHelper>(new EuslispServiceMessageHelper(fncallback, message.__getMD5Sum(), message.__getDataType(), request, response)));
  AdvertiseServiceOptions aso(service, *callback);
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

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  ckarg(2);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);
  if ( isstring(argv[1]) ) {
    s.assign((char *)get_string(argv[1]));
    s_node->setParam(key,s);
  } else if (isint(argv[1])) {
    i = intval(argv[1]);
    s_node->setParam(key,i);
  } else if (isflt(argv[1])) {
    d = fltval(argv[1]);
    s_node->setParam(key,d);
  } else {
    error(E_MISMATCHARG);
  }
  return (T);
}

pointer ROSEUS_GET_PARAM(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;
  ros::NodeHandle nh("~");

  ckarg(1);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }
  
  string s;
  double d;
  int i;
  pointer ret;

  if ( nh.getParam(key, s) ) {
    ret = makestring((char *)s.c_str(), s.length());
  } else if ( nh.getParam(key, d) ) {
    ret = makeflt(d);
  } else if ( nh.getParam(key, i) ) {
    ret = makeint(i);
  } else {
    return (NIL);
  }
  return (ret);
}

pointer ROSEUS_GET_PARAM_CASHED(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  string key;
  ros::NodeHandle nh("~");
  ckarg(1);
  if (isstring(argv[0])) key.assign((char *)get_string(argv[0]));
  else error(E_NOSTRING);

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  string s;
  double d;
  int i;
  pointer ret;

  if ( nh.getParamCached(key, s) ) {
    ret = makestring((char *)s.c_str(), s.length());
  } else if ( nh.getParamCached(key, d) ) {
    ret = makeflt(d);
  } else if ( nh.getParamCached(key, i) ) {
    ret = makeint(i);
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

  if( !s_node ) {
    ROS_ERROR("could not find node handle");
    return (NIL);
  }

  return((s_node->hasParam(key))?(T):(NIL));
}

/************************************************************
 *   __roseus
 ************************************************************/

pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("ROS", 3),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  QANON=defvar(ctx,"*ANONYMOUS-NAME*",makeint(ros::init_options::AnonymousName),rospkg);
  QNOINT=defvar(ctx,"*NO-SIGINT-HANDLER*",makeint(ros::init_options::NoSigintHandler),rospkg);
  QNOOUT=defvar(ctx,"*NO-ROSOUT*",makeint(ros::init_options::NoRosout),rospkg);

  defun(ctx,"SPIN",argv[0],(pointer (*)())ROSEUS_SPIN);
  defun(ctx,"SPIN-ONCE",argv[0],(pointer (*)())ROSEUS_SPINONCE);
  defun(ctx,"TIME-NOW-RAW",argv[0],(pointer (*)())ROSEUS_TIME_NOW);
  defun(ctx,"RATE",argv[0],(pointer (*)())ROSEUS_RATE);
  defun(ctx,"SLEEP",argv[0],(pointer (*)())ROSEUS_SLEEP);
  defun(ctx,"OK",argv[0],(pointer (*)())ROSEUS_OK);

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

  pointer_update(Spevalof(PACKAGE),p);
  defun(ctx,"ROSEUS-RAW",argv[0],(pointer (*)())ROSEUS);

  return 0;
}

