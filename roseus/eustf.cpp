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

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string

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

pointer _EUSTF_LOOKUP_TRANSFORM(register context *ctx,int n,pointer *argv)
{
  //NB: currently, only support NOW
  pointer vs;
  tf::TransformListener tf;
  tf::StampedTransform ret_transform;
  // tf::lookup-transform(from-id to-if)
  // take 2 arguments
  ckarg(2);
  // check frame ids
  if ( !isstring(argv[0]) ) error(E_NOSTRING);
  if ( !isstring(argv[1]) ) error(E_NOSTRING);
  //TODO: Error Check
  std::string from_id_string = std::string((char*)(argv[0]->c.str.chars));
  std::string to_id_string = std::string((char*)(argv[1]->c.str.chars));
  tf.waitForTransform(from_id_string, to_id_string,
                      ros::Time(), ros::Duration(1.0));
  tf.lookupTransform(from_id_string, to_id_string, ros::Time(), ret_transform);

  // return as array, i dont know create coordinates in C...
  vs = makefvector(7);          //pos[3] + rot[4](angle-axis quaternion)
  tf::Vector3 p = ret_transform.getOrigin();
  tf::Quaternion q = ret_transform.getRotation();
  vs->c.fvec.fv[0] = p.getX();
  vs->c.fvec.fv[1] = p.getY();
  vs->c.fvec.fv[2] = p.getZ();
  vs->c.fvec.fv[3] = q.getW();
  vs->c.fvec.fv[4] = q.getX();
  vs->c.fvec.fv[5] = q.getY();
  vs->c.fvec.fv[6] = q.getZ();
  return(vs);
}



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
  return((pointer)(new tf::Transformer(interpolating, ros::Duration(cache_time))));
}

pointer EUSTF_ALLFRAMESASSTRING(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::Transformer *tf = (tf::Transformer *)argv[0];
  string str = tf->allFramesAsString();
  return(makestring((char *)(str.c_str()), str.length()));
}

pointer EUSTF_SETTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ROS_ERROR("%s is not implemented yet", __PRETTY_FUNCTION__);
  return(T);
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

  tf = (tf::Transformer *)argv[0];
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

  ret = tf->waitForTransform(target_frame, source_frame, time,
                             ros::Duration(timeout), ros::Duration(duration));

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

  tf = (tf::Transformer *)argv[0];
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

  ret = tf->waitForTransform(target_frame, target_time,
                             source_frame, source_time,
                             fixed_frame,
                             ros::Duration(timeout), ros::Duration(duration));

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  tf::Transformer *tf;
  std::string target_frame, source_frame;
  ros::Time time;
  bool ret;

  tf = (tf::Transformer *)argv[0];
  if (isstring(argv[1]))
    target_frame = std::string((char*)(argv[1]->c.str.chars)); 
  else error(E_NOSTRING);

  if (isstring(argv[2]))
    source_frame = std::string((char*)(argv[2]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(time,argv[3]);

  ret = tf->canTransform(target_frame, source_frame, time);

  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(7);
  tf::Transformer *tf;
  std::string target_frame, source_frame, fixed_frame;
  ros::Time target_time, source_time;
  bool ret;

  tf = (tf::Transformer *)argv[0];
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

  ret = tf->canTransform(target_frame, target_time,
                         source_frame, source_time,
                         fixed_frame);

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
  tf::Transformer *tf = (tf::Transformer *)argv[0];
  tf->clear();
  return(T);
}

pointer EUSTF_FRAMEEXISTS(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  tf::Transformer *tf;
  std::string frame_id;

  tf = (tf::Transformer *)argv[0];
  if (!isstring(argv[1])) error(E_NOSTRING);
  frame_id = std::string((char*)(argv[1]->c.str.chars));
  return(tf->frameExists(frame_id)?T:NIL);
}

pointer EUSTF_GETFRAMESTRINGS(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  tf::Transformer *tf = (tf::Transformer *)argv[0];
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

  tf = (tf::Transformer *)argv[0];
  if (!isstring(argv[1])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[1]->c.str.chars));
  if (!isstring(argv[2])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[2]->c.str.chars));

  ros::Time time;
  int r = tf->getLatestCommonTime(source_frame, target_frame, time, &error_string);
  if ( r == 0 ) {
    return(cons(ctx,makeint(time.sec),makeint(time.nsec)));
  } else {
    ROS_ERROR("%s", error_string.c_str());
    return(NIL);
  }
}

pointer EUSTF_LOOKUPTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  tf::Transformer *tf;
  std::string target_frame, source_frame;
  ros::Time time;

  tf = (tf::Transformer *)argv[0];
  if (!isstring(argv[1])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[1]->c.str.chars));
  if (!isstring(argv[2])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[2]->c.str.chars));

  set_ros_time(time,argv[3]);

  tf::StampedTransform transform;
  tf->lookupTransform(target_frame, source_frame, time, transform);

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

  tf = (tf::Transformer *)argv[0];
  if (!isstring(argv[1])) error(E_NOSTRING);
  target_frame = std::string((char*)(argv[1]->c.str.chars));

  set_ros_time(target_time,argv[2]);

  if (!isstring(argv[3])) error(E_NOSTRING);
  source_frame = std::string((char*)(argv[3]->c.str.chars));

  set_ros_time(source_time,argv[4]);

  if (!isstring(argv[5])) error(E_NOSTRING);
  fixed_frame = std::string((char*)(argv[5]->c.str.chars));

  tf::StampedTransform transform;
  tf->lookupTransform(target_frame, target_time,
                      source_frame, source_time, fixed_frame, transform);

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
  tf::TransformListener *tf = (tf::TransformListener *)argv[0];
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
  tf->transformPose(target_frame, input, output);
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

  tf = (tf::Transformer *)argv[0];
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

  geometry_msgs::TwistStamped velocity;
  ROS_ERROR("%s is not implemented yet since lookupVelocity seems obsoluted", __PRETTY_FUNCTION__);
  /* tf->lookupVelocity(reference_frame, moving_frame, ros::Time(time), ros::Time(duration), velocity); */

  pointer vs = makefvector(6);          //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  vs->c.fvec.fv[0] = velocity.twist.linear.x;
  vs->c.fvec.fv[1] = velocity.twist.linear.y;
  vs->c.fvec.fv[2] = velocity.twist.linear.z;
  vs->c.fvec.fv[3] = velocity.twist.angular.x;
  vs->c.fvec.fv[4] = velocity.twist.angular.x;
  vs->c.fvec.fv[5] = velocity.twist.angular.x;
  vpop();
  return(vs);
}

/* */
pointer EUSTF_TRANSFORM_LISTENER(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(2);
  float cache_time = ckfltval(argv[0]);
  bool spin_thread = ((argv[1]==T)?true:false);
  return((pointer)(new tf::TransformListener(ros::Duration(cache_time), spin_thread)));
}

/* */
pointer EUSTF_SETEXTRAPOLATIONLIMIT(register context *ctx,int n,pointer *argv)
{
  numunion nu;
  ckarg(2);
  tf::Transformer *tf;
  tf = (tf::Transformer *)argv[0];
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

  tf = (tf::Transformer *)argv[0];

  if (isstring(argv[1]))
    frame_id = std::string((char*)(argv[1]->c.str.chars));
  else error(E_NOSTRING);

  set_ros_time(time,argv[2]);

  bool ret = tf->getParent(frame_id, time, parent);

  return(ret?makestring((char *)parent.c_str(),parent.length()):NIL);
}

pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("TF",2));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("TF", 2),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;
  defun(ctx,"_LOOKUP-TRANSFORM",argv[0],(pointer (*)())_EUSTF_LOOKUP_TRANSFORM);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0 ) {
    ROS_ERROR("Coudld not found ROS package; Please load eusros.so");
    exit(2);
  }
  Spevalof(PACKAGE)=rospkg;
  defun(ctx,"EUSTF-TRANSFORMER",argv[0],(pointer (*)())EUSTF_TRANSFORMER);
  defun(ctx,"EUSTF-ALL-FRAMES-AS-STRING",argv[0],(pointer (*)())EUSTF_ALLFRAMESASSTRING);
  defun(ctx,"EUSTF-SETT-RANSFORM",argv[0],(pointer (*)())EUSTF_SETTRANSFORM);
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM",argv[0],(pointer (*)())EUSTF_WAITFORTRANSFORM);
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_WAITFORTRANSFORMFULL);
  defun(ctx,"EUSTF-CAN-TRANSFORM",argv[0],(pointer (*)())EUSTF_CANTRANSFORM);
  defun(ctx,"EUSTF-CAN-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_CANTRANSFORMFULL);
  defun(ctx,"EUSTF-CHAIN",argv[0],(pointer (*)())EUSTF_CHAIN);
  defun(ctx,"EUSTF-CLEAR",argv[0],(pointer (*)())EUSTF_CLEAR);
  defun(ctx,"EUSTF-FRAME-EXISTS",argv[0],(pointer (*)())EUSTF_FRAMEEXISTS);
  defun(ctx,"EUSTF-GET-FRAME-STRINGS",argv[0],(pointer (*)())EUSTF_GETFRAMESTRINGS);
  defun(ctx,"EUSTF-GET-LATEST-COMMON-TIME",argv[0],(pointer (*)())EUSTF_GETLATERSTCOMMONTIME);
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM",argv[0],(pointer (*)())EUSTF_LOOKUPTRANSFORM);
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM-FULL",argv[0],(pointer (*)())EUSTF_LOOKUPTRANSFORMFULL);
  defun(ctx,"EUSTF-TRANSFORM-POSE",argv[0],(pointer (*)())EUSTF_TRANSFORMPOSE);
  defun(ctx,"EUSTF-LOOKUP-VELOCITY",argv[0],(pointer (*)())EUSTF_LOOKUPVELOCITY);
  /* */
  defun(ctx,"EUSTF-TRANSFORM-LISTENER",argv[0],(pointer (*)())EUSTF_TRANSFORM_LISTENER);
  /* */
  defun(ctx,"EUSTF-SET-EXTRAPOLATION-LIMIT",argv[0],(pointer (*)())EUSTF_SETEXTRAPOLATIONLIMIT);
  defun(ctx,"EUSTF-GET-PARENT",argv[0],(pointer (*)())EUSTF_GETPARENT);

  pointer_update(Spevalof(PACKAGE),p);
  return 0;
}
