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

#include <ros/node.h>
#include <ros/service.h>
#include <ros/session.h>

// for eus.h
#define float_t eus_float_t
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

#undef float_t
#undef class
#undef throw
#undef export
#undef vector
#undef string

using namespace ros;
using namespace std;

// copy from rosoct.cpp
class RoscppWorker
{
public:
  virtual ~RoscppWorker() {}
  virtual void workerthread(void* userdata) = 0;
};

// executes a worker
class RoscppWorkExecutor
{
public:
  RoscppWorkExecutor(RoscppWorker* pworker, void* userdata) : _pworker(pworker), _userdata(userdata) {}
  virtual ~RoscppWorkExecutor() { _pworker->workerthread(_userdata); }
private:
  RoscppWorker* _pworker;
  void* _userdata;
};

void reset_all();
ros::Node* check_roscpp_nocreate();
void AddWorker(RoscppWorker* psub, void* userdata);
void __roseus_worker(int num);

// getstring, getdata, serialize, deserialize...

// it is necessary to call the octave serialize function directly since a sequence id is needed

class EuslispMsgSerializer : public Message
{
  string _md5sum, _type, _data;
  int _serlen;

public:
  EuslispMsgSerializer() : _serlen(0) {}
  EuslispMsgSerializer(const EuslispMsgSerializer& r) : Message()
  {
    _md5sum = r._md5sum;
    _type = r._type;
    _serlen = r._serlen;
    _data = r._data;
  }
  EuslispMsgSerializer(const string& md5sum, char *serptr, int serlen, const string& type)
  {
    _serlen = serlen;
    _type = type;
    _md5sum = md5sum;
    _data.resize(serlen);
    memcpy(&_data[0], serptr, _serlen);
  }
  virtual ~EuslispMsgSerializer() {}

  virtual const string __getDataType() const { return _type; }
  virtual const string __getMD5Sum()   const { return _md5sum; }
  virtual const string __getMessageDefinition()   const { return ""; }
  virtual const string __getServiceDataType() const { return ""; }
  virtual const string __getServerMD5Sum() const { return _md5sum; }

  uint32_t serializationLength() const { return _serlen; }

  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    unsigned __ros_data_len = _serlen;
    SROS_SERIALIZE_BUFFER(write_ptr, _data.c_str(), __ros_data_len);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *readPtr) { ROS_ASSERT(0); return NULL; }
};


class EuslispMsgDeserializer : public Message
{
public:
  EuslispMsgDeserializer() {}
  EuslispMsgDeserializer(const EuslispMsgDeserializer& r) : Message()
  {
    _type = r._type;
    _md5sum = r._md5sum;
    _vdata = r._vdata;
    __connection_header = r.__connection_header;
  }
  EuslispMsgDeserializer(const string& md5sum, const string& type = string("*"))
  {
    _type = type;
    _md5sum = md5sum;
  }

  string _md5sum, _type;
  vector<uint8_t> _vdata;
  boost::shared_ptr<ros::M_string> __connection_header;

  virtual const string __getDataType() const { return _type; }
  virtual const string __getMD5Sum()   const { return _md5sum; }
  virtual const string __getMessageDefinition()   const { return ""; }
  virtual const string __getServerMD5Sum() const { return _md5sum; }
  virtual const std::string __getServiceDataType() const { return ""; }

  uint32_t serializationLength() const { return _vdata.size(); }
  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
  {
    if( seqid != 0 )
      // if you get this message, define SERVICE_SERIALIZE_SEQID and change roseus_service_call.m and roseus_session_call.m
      ROS_DEBUG("ignoring service sequence id %d!", seqid);

    if( _vdata.size() > 0 )
      memcpy(writePtr, &_vdata[0], _vdata.size());
    return writePtr + _vdata.size();
  }
  virtual uint8_t *deserialize(uint8_t *readPtr)
  {
    _vdata.resize(__serialized_length);
    if( _vdata.size() > 0 )
      memcpy(&_vdata[0], readPtr, _vdata.size());
    return readPtr+__serialized_length;
  }
};

/*
 *
 */

class RoscppSubscription : public RoscppWorker
{
public:
  RoscppSubscription(Node* pnode, const string& topicname, const string& md5sum, const string& type, /*int (* pfn)(int)*/pointer pfn, int maxqueue = 1)
  {
    _bDropWork = false;

    _msg._md5sum = md5sum;
    _msg._type = type;
    _topicname = topicname;
    _fncallback = pfn;

    assert( pnode != NULL && pfn != NULL );

    if( !pnode->subscribe(_topicname, _msg, &RoscppSubscription::cb, this, maxqueue) )
      throw;
  }
  virtual ~RoscppSubscription()
  {
    ros::Node* pnode = check_roscpp_nocreate();
    if( pnode != NULL ) {
      if( !pnode->unsubscribe(_topicname) )
        ROS_WARN("failed to unsubscribe from %s", _topicname.c_str());

      __roseus_worker(0); // flush
    }
  }

  virtual void workerthread(void* userdata)
  {
    boost::shared_ptr<EuslispMsgDeserializer> pmsg((EuslispMsgDeserializer*)userdata);
    boost::mutex::scoped_lock lock(_mutex);
    dowork(pmsg.get());
  }

private:
  void cb()
  {
    if( _bDropWork )
      return;
    //boost::mutex::scoped_lock lockserv(_mutexService); // lock simultaneous service calls out
    boost::mutex::scoped_lock lock(_mutex);
    AddWorker(this, new EuslispMsgDeserializer(_msg));
  }

  virtual void dowork(EuslispMsgDeserializer* pmsg)
  {
    if( pmsg->_vdata.size() > 0 ) {
      //char buf[pmsg->_vdata.size()];
      //memcpy(buf,&pmsg->_vdata[0],pmsg->_vdata.size());
      //(*_fncallback)(10);
      context *ctx = euscontexts[thr_self()];
      vpush((pointer)makestring((char *)(&pmsg->_vdata[0]),pmsg->_vdata.size()));
      //ROS_INFO("ufuncall for  %s(%d@%d)", _topicname.c_str(), _fncallback, ctx);
      ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),_fncallback,(pointer)(ctx->vsp-1),NULL,1);
      //ufuncall(ctx,_fncallback,_fncallback,(pointer)(ctx->vsp-1),ctx->bindfp,1);
      //ufuncall(ctx,(ctx->callfp?ctx->callfp->form:NIL),_fncallback,(pointer)(ctx->vsp-1),ctx->bindfp,1);
      vpop();
    }
#if 0
    args.resize(1);
    if( pmsg->_vdata.size() > 0 ) {
      odata.resize_no_fill(pmsg->_vdata.size());
      memcpy(odata.fortran_vec(),&pmsg->_vdata[0],pmsg->_vdata.size());
    }
    args(0) = odata;
    octave_value_list retval = ovfncallback.user_function_value()->do_multi_index_op(0, args);
#endif
  }

  string _topicname;
  EuslispMsgDeserializer _msg;
  pointer _fncallback;
  boost::mutex _mutex;
  bool _bDropWork;
};

class RoseusStaticData
{
public:
  RoseusStaticData() : nSessionHandleId(1) {}
  ~RoseusStaticData() {
  }
  list<boost::shared_ptr<RoscppWorkExecutor> > listWorkerItems;
  map<string, pair<string, string> > mapAdvertised; ///< advertised topics

  map<string, boost::shared_ptr<RoscppSubscription> > subscriptions;
  //map<string, boost::shared_ptr<RoscppService> > services;
  //map<int, session::abstractSessionHandle> sessions;
  int nSessionHandleId; ///< counter of unique session ids to assign
  boost::mutex mutexWorker, mutexWorking;
  vector<char*> argv;
  vector<string> vargv;
};

static RoseusStaticData s_staticdata;

#define s_listWorkerItems s_staticdata.listWorkerItems
#define s_mapAdvertised s_staticdata.mapAdvertised
#define s_subscriptions s_staticdata.subscriptions
//#define s_services s_staticdata.services
//#define s_sessions s_staticdata.sessions
#define s_nSessionHandleId s_staticdata.nSessionHandleId
#define s_mutexWorker s_staticdata.mutexWorker
#define s_mutexWorking s_staticdata.mutexWorking

#define s_vargv s_staticdata.vargv

void AddWorker(RoscppWorker* psub, void* userdata)
{
  boost::mutex::scoped_lock lock(s_mutexWorker);
  s_listWorkerItems.push_back(boost::shared_ptr<RoscppWorkExecutor>(new RoscppWorkExecutor(psub, userdata)));
}

void __roseus_worker(int num)
{
  list<boost::shared_ptr<RoscppWorkExecutor> > listworkers;
  
  int nItemsToProcess = -1;
  if( num > 0 )
    nItemsToProcess = num;

  {
    boost::mutex::scoped_lock lock(s_mutexWorker);
    if( nItemsToProcess < 0 )
      listworkers.swap(s_listWorkerItems);
    else {
      nItemsToProcess = min(nItemsToProcess,(int)s_listWorkerItems.size());
      // only take nItemsToProcess from s_listWorkerItems
      list<boost::shared_ptr<RoscppWorkExecutor> >::iterator itlast = s_listWorkerItems.begin();
      advance(itlast,nItemsToProcess);
      listworkers.splice(listworkers.end(),s_listWorkerItems, s_listWorkerItems.begin(),itlast);
    }
  }
  
  boost::mutex::scoped_lock lock(s_mutexWorking);
  listworkers.clear(); // do all the work in the destructors
}

ros::Node* check_roscpp()
{
  // start roscpp
  ros::Node* pnode = ros::Node::instance();
  if( pnode && !pnode->checkMaster() ) {
    reset_all();

    delete pnode;
    return NULL;
  }

  if (!pnode) {
    char strname[256] = "nohost";
    gethostname(strname, sizeof(strname));
    strcat(strname,"_roseus");

    int argc = (int)s_vargv.size();
    vector<string> vargv = s_vargv;
    vector<char*> argv(vargv.size());
    for(size_t i = 0; i < argv.size(); ++i)
      argv[i] = &vargv[i][0];
    ros::init(argc,argv.size() > 0 ? &argv[0] : NULL);

    pnode = new ros::Node(strname, ros::Node::DONT_HANDLE_SIGINT|ros::Node::ANONYMOUS_NAME|ros::Node::DONT_ADD_ROSOUT_APPENDER);
    bool bCheckMaster = pnode->checkMaster();

    delete pnode;

    if( !bCheckMaster ) {
      ROS_INFO("ros not present");
      return NULL;
    }

    argc = (int)s_vargv.size();
    vargv = s_vargv;
    for(size_t i = 0; i < argv.size(); ++i)
      argv[i] = &vargv[i][0];
    ros::init(argc,argv.size() > 0 ? &argv[0] : NULL);
    pnode = new ros::Node(strname, ros::Node::DONT_HANDLE_SIGINT|ros::Node::ANONYMOUS_NAME);
    ROS_INFO("new roscpp node started");
  }

  return pnode;
}

//
ros::Node* check_roscpp_nocreate()
{
  ros::Node* pnode = ros::Node::instance();
  return (pnode && pnode->checkMaster()) ? pnode : NULL;
}

void reset_all()
{
  ROS_INFO("%s", __PRETTY_FUNCTION__);
  __roseus_worker(0); // flush all
  s_subscriptions.clear();
  //s_services.clear();
  //s_sessions.clear();

  ros::Node* pnode = check_roscpp_nocreate();
  if( pnode != NULL ) {
    for(map<string, pair<string,string> >::iterator it = s_mapAdvertised.begin(); it != s_mapAdvertised.end(); ++it)
      pnode->unadvertise(it->first);
  }
  s_mapAdvertised.clear();
}

static int roseus_hook(void)
{
  //BEGIN_INTERRUPT_IMMEDIATELY_IN_FOREIGN_CODE;
  //BEGIN_INTERRUPT_WITH_EXCEPTIONS;
  __roseus_worker(0); // flush all
  //END_INTERRUPT_WITH_EXCEPTIONS;
  //END_INTERRUPT_IMMEDIATELY_IN_FOREIGN_CODE;
  return 0;
}

int roseus_hook_thread (void) {
  while (1) {
    roseus_hook();
    usleep(100*1000);
  }
}

void roseus_exit()
{
  reset_all();
  if( ros::Node::instance() ) {
    delete ros::Node::instance();
  }
}
///
///
///
pointer ROSEUS_ADVERTISE(register context *ctx,int n,pointer *argv)
{ 
  string topicname, md5sum, type;
  int queuesize = 0;

  ckarg(4);
  if (isstring(argv[0])) topicname.assign((char *)(argv[0]->c.str.chars));
  else error(E_NOSTRING);
  if (isstring(argv[1]))  md5sum.assign((char *)(argv[1]->c.str.chars));
  else error(E_NOSTRING);
  if (isstring(argv[2]))  type.assign((char *)(argv[2]->c.str.chars));
  else error(E_NOSTRING);
  queuesize = ckintval(argv[3]);

  ros::Node* pnode = check_roscpp();
  if( !pnode ) {
    return (NIL);
  }

  if( s_mapAdvertised.find(topicname) != s_mapAdvertised.end() ) {
    fprintf(stderr, "\e[1;31mtopic already advertised\e[m\n");
    return (NIL);
  }

  EuslispMsgSerializer msgcloner(md5sum, "", 0, type);
  bool bSuccess = pnode->advertise(topicname, msgcloner, queuesize);

  if( bSuccess )
    s_mapAdvertised[topicname] = pair<string,string>(md5sum,type);
  
  return (T);
}

pointer ROSEUS_PUBLISH(register context *ctx,int n,pointer *argv)
{ 
  string topicname;
  char *msgstr = 0;
  int msglen;

  ckarg(3);
  if (isstring(argv[0])) topicname.assign((char *)(argv[0]->c.str.chars));
  else error(E_NOSTRING);
  if (isstring(argv[1])) msgstr = (char *)(argv[1]->c.str.chars);
  else error(E_NOSTRING);
  msglen = ckintval(argv[2]);

  ros::Node* pnode = check_roscpp();
  if( !pnode ) {
    return (NIL);
  }

  bool bSuccess = false;
  map<string,pair<string,string> >::iterator it = s_mapAdvertised.find(topicname);
  if( it != s_mapAdvertised.end() ) {
    EuslispMsgSerializer msgcloner(it->second.first, msgstr, msglen, it->second.second);
    pnode->publish(topicname, msgcloner);
    bSuccess = true;
  }
  
  if ( ! bSuccess ) {
    ROS_ERROR("attempted to publish to topic %s, which was not "        \
              "previously advertised. call (ros::advertise \"%s\") first.",
              topicname.c_str(), topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_SUBSCRIBE(register context *ctx,int n,pointer *argv)
{ 
  string topicname, md5sum, type;
  int queuesize = 1;
  pointer fncallback;

  ckarg(5);
  if (isstring(argv[0])) topicname.assign((char *)(argv[0]->c.str.chars));
  else error(E_NOSTRING);
  if (isstring(argv[1]))  md5sum.assign((char *)(argv[1]->c.str.chars));
  else error(E_NOSTRING);
  if (isstring(argv[2]))  type.assign((char *)(argv[2]->c.str.chars));
  else error(E_NOSTRING);
  fncallback = argv[3];
  queuesize = ckintval(argv[4]);
  
  ros::Node* pnode = check_roscpp();
  if( !pnode ) {
    return (NIL);
  }

  try {
    boost::shared_ptr<RoscppSubscription> subs(new RoscppSubscription(pnode, topicname, md5sum, type, fncallback, queuesize));
    s_subscriptions[topicname] = subs;

    ROS_INFO("subscribed to %s(%p@%p)", topicname.c_str(), fncallback, ctx);
  }
  catch(...) {
    // failed
    ROS_ERROR("failed to subscribe to %s", topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_EXIT(register context *ctx,int n,pointer *argv)
{ 
  roseus_exit();
  return (NIL);
}

/*
  not implimented yet...
 */

pointer CREATE_SESSION(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer SESSION_CALL(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer SERVICE_CALL(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer MSG_SUBSCRIBE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer MSG_UNSUBSCRIBE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer SET_PARAM(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer GET_PARAM(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer UNADVERTISE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer ADVERTISE_SERVICE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer UNADVERTISE_SERVICE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer TERMINATE_SESSION(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer GET_TOPICS(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer WAIT_FOR_SERVICE(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer CHECK_MASTER(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer WORKER(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

pointer TIME_NOW(register context *ctx,int n,pointer *argv)
{ 
  return (NIL);
}

bool install_roseus(bool bRegisterHook)
{
  if( bRegisterHook ) {
    //ROS_INFO("registering roseus hook");
    //boost::thread thr_event_hook(&roseus_hook_thread);
  }

  // register octave exit function
  //octave_value_list args; args.resize(1);
  //args(0) = "roseus_exit";
  //feval("atexit", args);
  return 1;
}

pointer ROSEUS(register context *ctx,int n,pointer *argv)
{ 
  static bool s_bInstalled = false;
  bool bRegisterHook = true;
  bool bSuccess = true;

  char *cmd = "";

  ckarg2(0,1);
  if (n == 1 ) {
    if (isstring(argv[0])) {
      cmd = (char *)(argv[0]->c.str.chars);
    } else {
      error(E_NOSTRING);
    }
  }

  ROS_INFO("ROSEUS %s", cmd);
  if ( strcmp(cmd, "clear") == 0 ) {
    reset_all();
  } else if ( strcmp(cmd, "shutdown") == 0 ) {
    reset_all();
    if( ros::Node::instance() ) {
      delete ros::Node::instance();
    }
  } else if( strcmp(cmd, "nohook") == 0 ) {
    bRegisterHook = false;
  } else {
    if( !s_bInstalled ) {
      if( install_roseus(bRegisterHook) )
        s_bInstalled = true;
      else {
        ROS_FATAL("roseus failed to initialize");
        bSuccess = false;
      }
    }
  }
    
  if( !s_bInstalled ) {
    if( install_roseus(bRegisterHook) )
      s_bInstalled = true;
    else {
      ROS_FATAL("roseus failed to initialize");
      bSuccess = false;
    }
  }

  return (bSuccess?T:NIL);
}

pointer ROSEUS_WORKER(register context *ctx,int n,pointer *argv)
{ 
  __roseus_worker(0); // flush all
  return (T);
}

pointer ___roseus(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("ROS", 3),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  defun(ctx,"ROSEUS_ADVERTISE",argv[0],(pointer (*)())ROSEUS_ADVERTISE);
  defun(ctx,"ROSEUS_PUBLISH",argv[0],(pointer (*)())ROSEUS_PUBLISH);
  defun(ctx,"ROSEUS_SUBSCRIBE",argv[0],(pointer (*)())ROSEUS_SUBSCRIBE);
  defun(ctx,"ROSEUS_WORKER",argv[0],(pointer (*)())ROSEUS_WORKER);
  defun(ctx,"ROSEUS_EXIT",argv[0],(pointer (*)())ROSEUS_EXIT);

  defun(ctx,"CREATE_SESSION",argv[0],(pointer (*)())CREATE_SESSION);
  defun(ctx,"SESSION_CALL",argv[0],(pointer (*)())SESSION_CALL);
  defun(ctx,"SERVICE_CALL",argv[0],(pointer (*)())SERVICE_CALL);
  defun(ctx,"MSG_UNSUBSCRIBE",argv[0],(pointer (*)())MSG_UNSUBSCRIBE);
  defun(ctx,"SET_PARAM",argv[0],(pointer (*)())SET_PARAM);
  defun(ctx,"GET_PARAM",argv[0],(pointer (*)())GET_PARAM);
  defun(ctx,"UNADVERTISE",argv[0],(pointer (*)())UNADVERTISE);
  defun(ctx,"ADVERTISE_SERVICE",argv[0],(pointer (*)())ADVERTISE_SERVICE);
  defun(ctx,"UNADVERTISE_SERVICE",argv[0],(pointer (*)())UNADVERTISE_SERVICE);
  defun(ctx,"TERMINATE_SESSION",argv[0],(pointer (*)())TERMINATE_SESSION);
  defun(ctx,"GET_TOPICS",argv[0],(pointer (*)())GET_TOPICS);
  defun(ctx,"WAIT_FOR_SERVICE",argv[0],(pointer (*)())WAIT_FOR_SERVICE);
  defun(ctx,"CHECK_MASTER",argv[0],(pointer (*)())CHECK_MASTER);
  defun(ctx,"TIME_NOW",argv[0],(pointer (*)())TIME_NOW);

  pointer_update(Spevalof(PACKAGE),p);
  defun(ctx,"ROSEUS",argv[0],(pointer (*)())ROSEUS);
  
  return 0;
}



