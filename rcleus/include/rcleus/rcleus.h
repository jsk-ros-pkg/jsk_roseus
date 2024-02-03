// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/
/*
 * rcleus.h
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef RCLEUS_H_
#define RCLEUS_H_

#include <cstdio>
#include <errno.h>
#include <list>
#include <map>
#include <math.h>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <setjmp.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>

// Define the EusLisp types not to conflict with the C++ types
#define class eus_class
#define throw eus_throw
#define export eus_export
#include <eus.h>
#undef class
#undef throw
#undef export

extern "C" {
  pointer ___rcleus(context *ctx, int n, pointer *argv, pointer env);
  void register_rcleus() {
    char modname[] = "___rcleus";
    return add_module_initializer(modname, (pointer(*)())___rcleus);
  }
}

struct RcleusStaticData {
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Rate> rate;
  // map<string, std::shared_ptr<Publisher>> mapAdvertised;  ///< advertised
  // topics map<string, std::shared_ptr<Subscriber>> mapSubscribed; ///<
  // subscribed topics map<string, std::shared_ptr<ServiceServer>>
  //     mapServiced;               ///< subscribed topics
  // map<string, Timer> mapTimered; ///< subscribed timers
  // map<string, boost::shared_ptr<NodeHandle>>
  //     mapHandle; ///< for grouping nodehandle
};

inline void rcleusSignalHandler(int sig) {
  // memoize for euslisp handler...
  context *ctx = euscontexts[thr_self()];
  ctx->intsig = sig;
}

// C implementations of rcleus
pointer RCLEUS(context *ctx, int n, pointer *argv);

#endif // RCLEUS_H_
