/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, JSK Lab, the Univ. of Tokyo
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

// author: Ryohei Ueda
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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/buffer_client.h>
#ifdef TF2_ROS_VERSION_3 // if this is groovy
#define tf2_ros tf2
#endif

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#ifdef __clang__
#undef MAX
#undef MIN
#endif
#include "eus.h"

extern "C" {
  pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env);
  void register_eustf(){
    char modname[] = "___eustf";
    return add_module_initializer(modname, (pointer (*)())___eustf);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

using namespace ros;
using namespace std;


/***********************************************************
 *   TF wrapper
 ************************************************************/

#define set_ros_time(time,argv)                         \
  if (isvector(argv) && (elmtypeof(argv)==ELM_INT)) {   \
    time.sec  = argv->c.ivec.iv[0];                     \
    time.nsec = argv->c.ivec.iv[1];                     \
  } else {                                              \
    error(E_NOVECTOR);                                  \
  }

/* */
pointer EUSTF_TRANSFORMER(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(2);
  bool interpolating = ((argv[0]==T)?true:false);
  float cache_time = ckfltval(argv[1]);
  return(makeint((eusinteger_t)(new tf::Transformer(interpolating, ros::Duration(cache_time)))));
}

pointer EUSTF_ALLFRAMESASSTRING(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::Transformer *tf = (tf::Transformer *)(intval(argv[0]));
  string str = tf->allFramesAsString();
  return(makestring((char *)(str.c_str()), str.length()));
}

pointer EUSTF_SETTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(7);
  tf::Transformer *tf = (tf::Transformer *)(intval(argv[0]));
  if (!isvector(argv[1])) error(E_NOVECTOR);
  if (!isvector(argv[2])) error(E_NOVECTOR);
  eusfloat_t *pos = argv[1]->c.fvec.fv;
  eusfloat_t *rot = argv[2]->c.fvec.fv;
  isintvector(argv[3]);
  eusinteger_t *stamp = argv[3]->c.ivec.iv;
  if (!isstring(argv[4])) error(E_NOSTRING);
  if (!isstring(argv[5])) error(E_NOSTRING);
  if (!isstring(argv[6])) error(E_NOSTRING);
  std::string frame_id = std::string((char*)(argv[4]->c.str.chars));
  std::string child_frame_id = std::string((char*)(argv[5]->c.str.chars));
  std::string authority= std::string((char*)(argv[6]->c.str.chars));
  tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
  transform.setRotation(tf::Quaternion(rot[3], rot[0], rot[1], rot[2]));
  transform.frame_id_ = frame_id;
  transform.child_frame_id_ = child_frame_id;
  transform.stamp_.sec = stamp[0];
  transform.stamp_.nsec = stamp[1];
  bool ret = tf->setTransform(transform, authority);
  return(ret?T:NIL);
}

pointer EUSTF_WAITFORTRANSFORM(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(6);
  tf::Transformer *tf;
  std::string target_frame, source_frame;
  ros::Time time;
  float timeout = 0, duration = 0;
  bool ret;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (isstring(argv[1]))
    target_frame = std::string((char*)(argv[1]->c.str.chars));
  else error(E_NOSTRING);

  if (isstring(argv[2]))
    source_frame = std::string((char*)(argv[2]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(time,argv[3]);

  if (isint(argv[4])) timeout = (float)intval(argv[4]);
  else if (isflt(argv[4])) timeout = (float)fltval(argv[4]);
  else error(E_NONUMBER);

  if (isint(argv[5])) duration = (float)intval(argv[5]);
  else if (isflt(argv[5])) duration = (float)fltval(argv[5]);
  else error(E_NONUMBER);

  std::string err_str = std::string();
  ret = tf->waitForTransform(target_frame, source_frame, time,
                             ros::Duration(timeout), ros::Duration(duration),
                             &err_str);
  if(!ret) {
    ROS_WARN_STREAM("waitForTransform failed! : " << err_str);
  }
  ROS_DEBUG_STREAM("waitForTransform : "
                   << "target_frame : " << target_frame
                   << "source_frame : " << source_frame
                   << "time : " << time
                   << "timeout : " << timeout
                   << "duration : " << duration
                   << "return : " << ret);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_WAITFORTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(8);
  tf::Transformer *tf;
  std::string target_frame, source_frame, fixed_frame;
  ros::Time target_time, source_time;
  float timeout = 0, duration = 0;
  bool ret;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (isstring(argv[1]))
    target_frame = std::string((char*)(argv[1]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(target_time,argv[2]);

  if (isstring(argv[3]))
    source_frame = std::string((char*)(argv[3]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(source_time,argv[4]);

  if (isstring(argv[5]))
    fixed_frame = std::string((char*)(argv[5]->c.str.chars));
  else error(E_NOSTRING);

  if (isint(argv[6])) timeout = (float)intval(argv[6]);
  else if (isflt(argv[6])) timeout = (float)fltval(argv[6]);
  else error(E_NONUMBER);

  if (isint(argv[7])) duration = (float)intval(argv[7]);
  else if (isflt(argv[7])) duration = (float)fltval(argv[7]);
  else error(E_NONUMBER);

  std::string err_str = std::string();
  ret = tf->waitForTransform(target_frame, target_time,
                             source_frame, source_time,
                             fixed_frame,
                             ros::Duration(timeout), ros::Duration(duration),
                             &err_str);
  if(!ret) {
    ROS_WARN_STREAM("waitForTransformFull failed! : " << err_str);
  }
  ROS_DEBUG_STREAM("waitForTransformFull : "
                   << "target_frame : " << target_frame
                   << "target_time : " << target_time
                   << "source_frame : " << source_frame
                   << "source_time : " << source_time
                   << "fixed_frame : " << fixed_frame
                   << "timeout : " << timeout
                   << "duration : " << duration
                   << "return : " << ret);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  tf::Transformer *tf;
  std::string target_frame, source_frame;
  ros::Time time;
  bool ret;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (isstring(argv[1]))
    target_frame = std::string((char*)(argv[1]->c.str.chars)); 
  else error(E_NOSTRING);

  if (isstring(argv[2]))
    source_frame = std::string((char*)(argv[2]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(time,argv[3]);

  std::string err_str = std::string();
  ret = tf->canTransform(target_frame, source_frame, time, &err_str);
  if(!ret) {
    ROS_WARN_STREAM("canTransform " << target_frame << " " << source_frame << " failed! : " << err_str);
  }
  ROS_DEBUG_STREAM("canTransform : "
                   << "target_frame : " << target_frame
                   << "source_frame : " << source_frame
                   << "time : " << time
                   << "return : " << ret);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(7);
  tf::Transformer *tf;
  std::string target_frame, source_frame, fixed_frame;
  ros::Time target_time, source_time;
  bool ret;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (isstring(argv[1]))
    target_frame = std::string((char*)(argv[1]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(target_time,argv[3]);

  if (isstring(argv[3]))
    source_frame = std::string((char*)(argv[3]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(source_time,argv[4]);

  if (isstring(argv[5]))
    fixed_frame = std::string((char*)(argv[5]->c.str.chars));
  else error(E_NOSTRING);

  std::string err_str = std::string();
  ret = tf->canTransform(target_frame, target_time,
                         source_frame, source_time,
                         fixed_frame, &err_str);
  if(!ret) {
    ROS_WARN_STREAM("canTransformFull " << target_frame << " " << source_frame << " failed! : " << err_str);
  }
  ROS_DEBUG_STREAM("canTransformFull : "
                   << "target_frame : " << target_frame
                   << "target_time : " << target_time
                   << "source_frame : " << source_frame
                   << "source_time : " << source_time
                   << "fixed_frame : " << fixed_frame
                   << "return : " << ret);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CHAIN(register context *ctx,int n,pointer *argv)
{
  ROS_ERROR("%s is not implemented yet", __PRETTY_FUNCTION__);
  return(T);
}

pointer EUSTF_CLEAR(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::Transformer *tf = (tf::Transformer *)(intval(argv[0]));
  tf->clear();
  return(T);
}

pointer EUSTF_FRAMEEXISTS(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  tf::Transformer *tf;
  std::string frame_id;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  frame_id = std::string((char*)(argv[1]->c.str.chars));
  return(tf->frameExists(frame_id)?T:NIL);
}

pointer EUSTF_GETFRAMESTRINGS(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::Transformer *tf = (tf::Transformer *)(intval(argv[0]));
  std::vector< std::string > ids;
  pointer str = NIL;

  tf->getFrameStrings(ids);
  for (std::vector< std::string >::iterator s = ids.begin(); s != ids.end(); s++) {
    str=cons(ctx,makestring((char *)(s->c_str()),s->length()),str);
  }

  return(str);
}

pointer EUSTF_GETLATERSTCOMMONTIME(register context *ctx,int n,pointer *argv)
{
  ckarg(3);
  tf::Transformer *tf;
  std::string source_frame, target_frame, error_string;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[1]->c.str.chars));
  if (!isstring(argv[2])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[2]->c.str.chars));

  ros::Time time;
  int r = tf->getLatestCommonTime(source_frame, target_frame, time, &error_string);
  if ( r == 0 ) {
    return(cons(ctx,makeint(time.sec),makeint(time.nsec)));
  } else {
    ROS_ERROR_STREAM("getLatestCommonTime " << target_frame << " " << source_frame << " failed! : " << error_string);
    return(NIL);
  }
}

pointer EUSTF_LOOKUPTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  tf::Transformer *tf;
  std::string target_frame, source_frame;
  ros::Time time;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[1]->c.str.chars));
  if (!isstring(argv[2])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[2]->c.str.chars));

  set_ros_time(time,argv[3]);

  tf::StampedTransform transform;
  try {
    tf->lookupTransform(target_frame, source_frame, time, transform);
  } catch ( std::runtime_error e ) {
    ROS_ERROR("%s",e.what()); return(NIL);
  }

  pointer vs = makefvector(7);          //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  tf::Vector3 p = transform.getOrigin();
  tf::Quaternion q = transform.getRotation();
  vs->c.fvec.fv[0] = p.getX();
  vs->c.fvec.fv[1] = p.getY();
  vs->c.fvec.fv[2] = p.getZ();
  vs->c.fvec.fv[3] = q.getW();
  vs->c.fvec.fv[4] = q.getX();
  vs->c.fvec.fv[5] = q.getY();
  vs->c.fvec.fv[6] = q.getZ();
  vpop();
  return(vs);
}

pointer EUSTF_LOOKUPTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(6);
  tf::Transformer *tf;
  std::string target_frame, source_frame, fixed_frame;
  ros::Time target_time, source_time;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[1]->c.str.chars));

  set_ros_time(target_time,argv[2]);

  if (!isstring(argv[3])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[3]->c.str.chars));

  set_ros_time(source_time,argv[4]);

  if (!isstring(argv[5])) error(E_NOSTRING);
  fixed_frame = std::string((char*)(argv[5]->c.str.chars));

  tf::StampedTransform transform;
  try {
    tf->lookupTransform(target_frame, target_time,
                        source_frame, source_time, fixed_frame, transform);
  } catch ( std::runtime_error e ) {
    ROS_ERROR("%s",e.what()); return(NIL);
  }

  pointer vs = makefvector(7);          //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  tf::Vector3 p = transform.getOrigin();
  tf::Quaternion q = transform.getRotation();
  vs->c.fvec.fv[0] = p.getX();
  vs->c.fvec.fv[1] = p.getY();
  vs->c.fvec.fv[2] = p.getZ();
  vs->c.fvec.fv[3] = q.getW();
  vs->c.fvec.fv[4] = q.getX();
  vs->c.fvec.fv[5] = q.getY();
  vs->c.fvec.fv[6] = q.getZ();
  vpop();
  return(vs);
}


pointer EUSTF_TRANSFORMPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(5);                     // tf, target_frame, time, frame_id, 
                                // pose as float-vector. its vector is a vector
                                // appended position and angle-vector quaternion
  tf::TransformListener *tf = (tf::TransformListener *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  std::string target_frame = std::string((char*)(argv[1]->c.str.chars));
  ros::Time tm;
  set_ros_time(tm, argv[2]);
  
  if (!isstring(argv[3])) error(E_NOSTRING);
  std::string frame_id = std::string((char*)(argv[3]->c.str.chars));
  
  geometry_msgs::PoseStamped input, output;
  // setup input
  input.pose.position.x = argv[4]->c.fvec.fv[0];
  input.pose.position.y = argv[4]->c.fvec.fv[1];
  input.pose.position.z = argv[4]->c.fvec.fv[2];
  input.pose.orientation.w = argv[4]->c.fvec.fv[3]; // angle-vector format
  input.pose.orientation.x = argv[4]->c.fvec.fv[4];
  input.pose.orientation.y = argv[4]->c.fvec.fv[5];
  input.pose.orientation.z = argv[4]->c.fvec.fv[6];
  //  input.header.
  input.header.stamp = tm;
  input.header.frame_id = frame_id;

  try {
    tf->transformPose(target_frame, input, output);
  } catch ( std::runtime_error e ) {
    ROS_ERROR("%s",e.what()); return(NIL);
  }

  pointer vs = makefvector(7); //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  vs->c.fvec.fv[0] = output.pose.position.x;
  vs->c.fvec.fv[1] = output.pose.position.y;
  vs->c.fvec.fv[2] = output.pose.position.z;
  vs->c.fvec.fv[3] = output.pose.orientation.w;
  vs->c.fvec.fv[4] = output.pose.orientation.x;
  vs->c.fvec.fv[5] = output.pose.orientation.y;
  vs->c.fvec.fv[6] = output.pose.orientation.z;
  vpop();
  return(vs);
}

pointer EUSTF_LOOKUPVELOCITY(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(4);
  tf::Transformer *tf;
  std::string reference_frame, moving_frame;
  float time = 0, duration = 0;

  tf = (tf::Transformer *)(intval(argv[0]));
  if (!isstring(argv[1])) error(E_NOSTRING);
  reference_frame = std::string((char*)(argv[1]->c.str.chars));

  if (!isstring(argv[2])) error(E_NOSTRING);
  moving_frame = std::string((char*)(argv[2]->c.str.chars));

  if (isint(argv[3])) time = (float)intval(argv[3]);
  else if (isflt(argv[3])) time = (float)fltval(argv[3]);
  else error(E_NONUMBER);

  if (isint(argv[4])) duration = (float)intval(argv[4]);
  else if (isflt(argv[4])) duration = (float)fltval(argv[4]);
  else error(E_NONUMBER);

  geometry_msgs::Twist velocity;
  // ROS_ERROR("%s is not implemented yet since lookupVelocity seems obsoluted", __PRETTY_FUNCTION__);
  tf->lookupTwist(reference_frame, moving_frame, ros::Time(time), ros::Duration(duration), velocity);

  pointer vs = makefvector(6);          //pos[3] + rot[3](angle-axis)
  vpush(vs);
  vs->c.fvec.fv[0] = velocity.linear.x;
  vs->c.fvec.fv[1] = velocity.linear.y;
  vs->c.fvec.fv[2] = velocity.linear.z;
  vs->c.fvec.fv[3] = velocity.angular.x;
  vs->c.fvec.fv[4] = velocity.angular.y;
  vs->c.fvec.fv[5] = velocity.angular.z;
  vpop();
  return(vs);
}

/* */
pointer EUSTF_TRANSFORM_LISTENER(register context *ctx,int n,pointer *argv)
{
  if( ! ros::ok() ) { error(E_USER,"You must call ros::init() before creating the first NodeHandle"); }
  numunion nu;
  ckarg(2);
  float cache_time = ckfltval(argv[0]);
  bool spin_thread = ((argv[1]==T)?true:false);
  return(makeint((eusinteger_t)(new tf::TransformListener(ros::Duration(cache_time), spin_thread))));
}

pointer EUSTF_TRANSFORM_LISTENER_DISPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::TransformListener *tf = (tf::TransformListener *)(intval(argv[0]));
  delete(tf);
  return(T);
}

/* */
pointer EUSTF_SETEXTRAPOLATIONLIMIT(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(2);
  tf::Transformer *tf;
  tf = (tf::Transformer *)(intval(argv[0]));
  float distance = ckfltval(argv[1]);

  tf->setExtrapolationLimit(ros::Duration(distance));
  return(T);
}

pointer EUSTF_GETPARENT(register context *ctx,int n,pointer *argv)
{
  ckarg(3);
  tf::Transformer *tf;
  std::string frame_id, parent;
  ros::Time time;

  tf = (tf::Transformer *)(intval(argv[0]));

  if (isstring(argv[1]))
    frame_id = std::string((char*)(argv[1]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(time,argv[2]);

  try {
    bool ret = tf->getParent(frame_id, time, parent);
    return(ret?makestring((char *)parent.c_str(),parent.length()):NIL);
  } catch ( std::runtime_error e ) {
    ROS_ERROR("%s",e.what()); return(NIL);
  }
}

/* */
pointer EUSTF_TRANSFORM_BROADCASTER(register context *ctx,int n,pointer *argv)
{
  if( ! ros::ok() ) { error(E_USER,"You must call ros::init() before creating the first NodeHandle"); }
  return(makeint((eusinteger_t)(new tf::TransformBroadcaster())));
}

pointer EUSTF_SEND_TRANSFORM(register context *ctx,int n,pointer *argv){

  /* ptr pos quarternion parent_frame_id, child_frame_id, time */
  ckarg(6);

  tf::TransformBroadcaster *bc = (tf::TransformBroadcaster *)(intval(argv[0]));

  isintvector(argv[5]);
  ros::Time tm;
  tm.sec = argv[5]->c.ivec.iv[0];
  tm.nsec = argv[5]->c.ivec.iv[1];

  eusfloat_t *pos, *quaternion;
  std::string p_frame_id, c_frame_id;
  isfltvector(argv[1]);
  isfltvector(argv[2]);
  isstring(argv[3]);
  isstring(argv[4]);
  pos = argv[1]->c.fvec.fv;
  quaternion= argv[2]->c.fvec.fv;
  p_frame_id = (char *)argv[3]->c.str.chars;
  c_frame_id = (char *)argv[4]->c.str.chars;

  geometry_msgs::TransformStamped trans;
  trans.header.stamp = tm;
  trans.header.frame_id = p_frame_id;
  trans.child_frame_id = c_frame_id;
  trans.transform.translation.x = pos[0]/1000.0;
  trans.transform.translation.y = pos[1]/1000.0;
  trans.transform.translation.z = pos[2]/1000.0;

  trans.transform.rotation.w = quaternion[0];
  trans.transform.rotation.x = quaternion[1];
  trans.transform.rotation.y = quaternion[2];
  trans.transform.rotation.z = quaternion[3];

  bc->sendTransform(trans);

  return (T);
}

/* tf2 */
pointer EUSTF_BUFFER_CLIENT(register context *ctx,int n,pointer *argv)
{
  if(!ros::ok()) { error(E_USER, "You must call (ros::roseus \"nodename\") before creating the first NodeHandle"); }
  /* &optional (ns "tf2_buffer_server") (check_frequency 10.0) (timeout_padding 2.0) */
  numunion nu;
  std::string ns_str ("tf2_buffer_server");
  double check_frequency = 10.0;
  ros::Duration timeout_padding(2.0);

  ckarg2(0, 3);
  if (n > 0) {
    if (isstring (argv[0])) {
      ns_str.assign ((char *)(argv[0]->c.str.chars));
    } else {
      error(E_NOSTRING);
    }
  }
  if (n > 1) {
    check_frequency = ckfltval(argv[1]);
  }
  if (n > 2) {
    eusfloat_t pd = ckfltval(argv[2]);
    timeout_padding = ros::Duration(pd);
  }

  return(makeint((eusinteger_t)(new tf2_ros::BufferClient (ns_str, check_frequency, timeout_padding))));
}

pointer EUSTF_BUFFER_CLIENT_DISPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  delete(tfbc);
  return(T);
}

pointer EUSTF_TFBC_WAITFORSERVER(register context *ctx,int n,pointer *argv)
{
  ckarg2(1, 2);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  numunion nu;
  bool ret;
  if (n > 1) {
    ros::Duration tm(ckfltval(argv[1]));
    ret = tfbc->waitForServer(tm);
  } else {
    ret = tfbc->waitForServer();
  }
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_TFBC_CANTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg2(4, 5);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  numunion nu;
  std::string target_frame, source_frame;
  ros::Time time;
  ros::Duration timeout(0.0);
  bool ret;

  if (isstring(argv[1])) {
    char *cstr = (char*)(argv[1]->c.str.chars);
    if (cstr[0] == '/') {
      target_frame.assign((char *)(cstr+1));
    } else {
      target_frame.assign(cstr);
    }
  } else error(E_NOSTRING);

  if (isstring(argv[2])) {
    char *cstr = (char*)(argv[2]->c.str.chars);
    if (cstr[0] == '/') {
      source_frame.assign((char *)(cstr+1));
    } else {
      source_frame.assign(cstr);
    }
  } else error(E_NOSTRING);

  set_ros_time(time, argv[3]);

  if (n > 4) {
    timeout = ros::Duration(ckfltval(argv[4]));
  }
  //target_frame.
  std::string err_str = std::string();
  ret = tfbc->canTransform(target_frame, source_frame, time, timeout, &err_str);
  if(!ret) {
    ROS_WARN_STREAM("BufferClient::waitForTransform failed! : " << err_str);
  }
  ROS_DEBUG_STREAM("BufferClient::waitForTransform : "
                   << "target_frame : " << target_frame
                   << "source_frame : " << source_frame
                   << "time : " << time
                   << "timeout : " << timeout
                   << "return : " << ret);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_TFBC_LOOKUPTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg2(4, 5);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  numunion nu;
  std::string target_frame, source_frame;
  ros::Time time;
  ros::Duration timeout(0.0);

  if (!isstring(argv[1])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[1]->c.str.chars));

  if (!isstring(argv[2])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[2]->c.str.chars));

  set_ros_time(time, argv[3]);

  if (n > 4) {
    timeout = ros::Duration(ckfltval(argv[4]));
  }

  geometry_msgs::TransformStamped trans;
  try {
    trans = tfbc->lookupTransform(target_frame, source_frame, time, timeout);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what()); return(NIL);
  }

  pointer vs = makefvector(7);  //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  //trans.header.frame_id
  //trans.child_frame_id
  //trans.header.stamp --> ??
  vs->c.fvec.fv[0] = trans.transform.translation.x;
  vs->c.fvec.fv[1] = trans.transform.translation.y;
  vs->c.fvec.fv[2] = trans.transform.translation.z;
  vs->c.fvec.fv[3] = trans.transform.rotation.w;
  vs->c.fvec.fv[4] = trans.transform.rotation.x;
  vs->c.fvec.fv[5] = trans.transform.rotation.y;
  vs->c.fvec.fv[6] = trans.transform.rotation.z;
  vpop();

  return(vs);
}

#include "defun.h"
pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("TF",2));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("TF", 2),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0 ) {
    ROS_ERROR("Coudld not found ROS package; Please load eusros.so");
    exit(2);
  }
  Spevalof(PACKAGE)=rospkg;
  defun(ctx,"EUSTF-TRANSFORMER",argv[0],(pointer (*)())EUSTF_TRANSFORMER,"A Class which provides coordinate transforms between any two frames in a system");
  defun(ctx,"EUSTF-ALL-FRAMES-AS-STRING",argv[0],(pointer (*)())EUSTF_ALLFRAMESASSTRING,"A way to see what frames have been cached Useful for debugging");
  defun(ctx,"EUSTF-SET-TRANSFORM",argv[0],(pointer (*)())EUSTF_SETTRANSFORM,"Add transform information to the tf data structure");
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM",argv[0],(pointer (*)())EUSTF_WAITFORTRANSFORM,"Block until a transform is possible or it times out");
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_WAITFORTRANSFORMFULL,"Block until a transform is possible or it times out");
  defun(ctx,"EUSTF-CAN-TRANSFORM",argv[0],(pointer (*)())EUSTF_CANTRANSFORM,"Test if a transform is possible");
  defun(ctx,"EUSTF-CAN-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_CANTRANSFORMFULL,"Test if a transform is possible");
  defun(ctx,"EUSTF-CHAIN",argv[0],(pointer (*)())EUSTF_CHAIN,"Debugging function that will print the spanning chain of transforms");
  defun(ctx,"EUSTF-CLEAR",argv[0],(pointer (*)())EUSTF_CLEAR,"Clear all data");
  defun(ctx,"EUSTF-FRAME-EXISTS",argv[0],(pointer (*)())EUSTF_FRAMEEXISTS,"Check if a frame exists in the tree");
  defun(ctx,"EUSTF-GET-FRAME-STRINGS",argv[0],(pointer (*)())EUSTF_GETFRAMESTRINGS,"A way to get a std::vector of available frame ids");
  defun(ctx,"EUSTF-GET-LATEST-COMMON-TIME",argv[0],(pointer (*)())EUSTF_GETLATERSTCOMMONTIME,"Return the latest rostime which is common across the spanning set zero if fails to cross");
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM",argv[0],(pointer (*)())EUSTF_LOOKUPTRANSFORM,"Get the transform between two frames by frame ID");
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_LOOKUPTRANSFORMFULL,"Get the transform between two frames by frame ID");
  defun(ctx,"EUSTF-TRANSFORM-POSE",argv[0],(pointer (*)())EUSTF_TRANSFORMPOSE,"Transform a Stamped Pose into the target frame This can throw anything a lookupTransform can throw as well as tf::InvalidArgument");
  defun(ctx,"EUSTF-LOOKUP-VELOCITY",argv[0],(pointer (*)())EUSTF_LOOKUPVELOCITY,"Lookup the twist of the tracking_frame with respect to the observation frame in the reference_frame using the reference point");
  /* */
  defun(ctx,"EUSTF-TRANSFORM-LISTENER",argv[0],(pointer (*)())EUSTF_TRANSFORM_LISTENER,"This class inherits from Transformer and automatically subscribes to ROS transform messages");
  defun(ctx,"EUSTF-TRANSFORM-LISTENER-DISPOSE",argv[0],(pointer (*)())EUSTF_TRANSFORM_LISTENER_DISPOSE,"Destructor for TransformListener");

  /* */
  defun(ctx,"EUSTF-SET-EXTRAPOLATION-LIMIT",argv[0],(pointer (*)())EUSTF_SETEXTRAPOLATIONLIMIT,"Set the distance which tf is allow to extrapolate");
  defun(ctx,"EUSTF-GET-PARENT",argv[0],(pointer (*)())EUSTF_GETPARENT,"Fill the parent of a frame");
  /* */
  defun(ctx,"EUSTF-TRANSFORM-BROADCASTER",argv[0],(pointer (*)())EUSTF_TRANSFORM_BROADCASTER,"This class provides an easy way to publish coordinate frame transform information. It will handle all the messaging and stuffing of messages. And the function prototypes lay out all the necessary data needed for each message");
  defun(ctx,"EUSTF-SEND-TRANSFORM",argv[0],(pointer (*)())EUSTF_SEND_TRANSFORM,"Send a StampedTransform The stamped data structure includes frame_id, and time, and parent_id already");

  /* tf2 */
  defun(ctx,"EUSTF-BUFFER-CLIENT",argv[0],(pointer (*)())EUSTF_BUFFER_CLIENT,"Action client-based implementation of the tf2_ros::BufferInterface abstract data type. BufferClient uses actionlib to coordinate waiting for available transforms.");
  defun(ctx,"EUSTF-BUFFER-CLIENT-DISPOSE",argv[0],(pointer (*)())EUSTF_BUFFER_CLIENT_DISPOSE,"tf2 BufferClient destructor");
  defun(ctx,"EUSTF-TF2-WAIT-FOR-SERVER",argv[0],(pointer (*)())EUSTF_TFBC_WAITFORSERVER,"Block until the action server is ready to respond to requests");
  defun(ctx,"EUSTF-TF2-CAN-TRANSFORM",argv[0],(pointer (*)())EUSTF_TFBC_CANTRANSFORM,"Test if a transform is possible");
  defun(ctx,"EUSTF-TF2-LOOKUP-TRANSFORM",argv[0],(pointer (*)())EUSTF_TFBC_LOOKUPTRANSFORM,"Get the transform between two frames by frame ID");

  pointer_update(Spevalof(PACKAGE),p);
  return 0;
}
