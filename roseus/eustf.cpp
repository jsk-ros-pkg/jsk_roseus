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
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"

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

pointer EUSTF_LOOKUP_TRANSFORM(register context *ctx,int n,pointer *argv)
{
  //NB: currently, only support NOW
  char from_id[256] = "";
  char to_id[256] = "";
  int frame_time;
  pointer vs;
  numunion nu;                  // for makeflt
  tf::TransformListener tf;
  tf::StampedTransform ret_transform;
  // tf::lookup-transform(from-id to-if)
  // take 2 arguments
  ckarg(2);
  // store ids
  if ( isstring(argv[0]) )
    strncpy(from_id, (char*)(argv[0]->c.str.chars),255);
  else
    error(E_NOSTRING);
  if ( isstring(argv[1]) )
    strncpy(to_id, (char*)(argv[1]->c.str.chars),255);
  else
    error(E_NOSTRING);
  //TODO: Error Check
  std::string from_id_string = std::string(from_id);
  std::string to_id_string = std::string(to_id);
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

pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("TF",2));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("TF", 2),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;
  defun(ctx,"_LOOKUP-TRANSFORM",argv[0],(pointer (*)())EUSTF_LOOKUP_TRANSFORM);
  pointer_update(Spevalof(PACKAGE),p);
  return 0;
}
