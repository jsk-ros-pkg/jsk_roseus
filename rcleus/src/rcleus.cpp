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
 * rcleus.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "rcleus/rcleus.h"

static RcleusStaticData s_staticData;
static bool s_bInstalled = false; // flag to check if the rclcpp is initialized

pointer RCLEUS(context *ctx, int n, pointer *argv) {
  char name[256] = "";
  uint32_t options = 0;
  int cargc = 0;
  char *cargv[32];

  if (s_bInstalled) {
    RCLCPP_WARN(rclcpp::get_logger(name), "RCLEUS is already installed as %s",
                name);
    return (T);
  }

  ckarg(3);
  if (isstring(argv[0]))
    strncpy(name, (char *)(argv[0]->c.str.chars), 255);
  else
    error(E_NOSTRING);
  options = ckintval(argv[1]);
  pointer p = argv[2];
  if (islist(p)) {
    while (1) {
      if (!iscons(p))
        break;
      cargv[cargc] = (char *)((ccar(p))->c.str.chars);
      cargc++;
      p = ccdr(p);
    }
  } else
    error(E_NOSEQ);

  // convert invalid node name charactors to _, we assume it is '-'
  for (unsigned int i = 0; i < strlen(name); i++)
    if (!(isalpha(name[i]) || isdigit(name[i])))
      name[i] = '_';

  // TODO defkeyword
  // K_ROSEUS_MD5SUM = defkeyword(ctx, "MD5SUM-");
  // K_ROSEUS_DATATYPE = defkeyword(ctx, "DATATYPE-");
  // K_ROSEUS_DEFINITION = defkeyword(ctx, "DEFINITION-");
  // K_ROSEUS_CONNECTION_HEADER =
  //     intern(ctx, "_CONNECTION-HEADER", 18, findpkg(makestring("ROS", 3)));
  // K_ROSEUS_SERIALIZATION_LENGTH = defkeyword(ctx, "SERIALIZATION-LENGTH");
  // K_ROSEUS_SERIALIZE = defkeyword(ctx, "SERIALIZE");
  // K_ROSEUS_DESERIALIZE = defkeyword(ctx, "DESERIALIZE");
  // K_ROSEUS_GET = defkeyword(ctx, "GET");
  // K_ROSEUS_INIT = defkeyword(ctx, "INIT");
  // K_ROSEUS_REQUEST = defkeyword(ctx, "REQUEST");
  // K_ROSEUS_RESPONSE = defkeyword(ctx, "RESPONSE");
  // K_ROSEUS_GROUPNAME = defkeyword(ctx, "GROUPNAME");
  // K_ROSEUS_ONESHOT = defkeyword(ctx, "ONESHOT");
  // K_ROSEUS_LAST_EXPECTED = defkeyword(ctx, "LAST-EXPECTED");
  // K_ROSEUS_LAST_REAL = defkeyword(ctx, "LAST-REAL");
  // K_ROSEUS_CURRENT_EXPECTED = defkeyword(ctx, "CURRENT-EXPECTED");
  // K_ROSEUS_CURRENT_REAL = defkeyword(ctx, "CURRENT-REAL");
  // K_ROSEUS_LAST_DURATION = defkeyword(ctx, "LAST-DURATION");
  // K_ROSEUS_SEC = defkeyword(ctx, "SEC");
  // K_ROSEUS_NSEC = defkeyword(ctx, "NSEC");

  // s_mapAdvertised.clear();
  // s_mapSubscribed.clear();
  // s_mapServiced.clear();
  // s_mapTimered.clear();
  // s_mapHandle.clear();

  /*
    set locale to none to let C RTL assume logging string can contain
    non-ascii characters. Refer:
    https://logging.apache.org/log4cxx/latest_stable/faq.html
  */
  setlocale(LC_ALL, "");

  try {
    rclcpp::init(cargc, cargv);
    s_staticData.node = std::make_shared<rclcpp::Node>(name);
  } catch (const rclcpp::exceptions::RCLError &e) {
    RCLCPP_ERROR(rclcpp::get_logger(name), "%s", e.what());
    error(E_MISMATCHARG);
    return (NIL);
  }

  // s_node.reset(new ros::NodeHandle());
  // s_rate.reset(new ros::Rate(50));

  s_bInstalled = true;

  // install signal handler for sigint. DO NOT call unix:signal after
  signal(SIGINT, rcleusSignalHandler);
  return (T);
}

pointer ___rcleus(context *ctx, int n, pointer *argv, pointer env) {
  defun(ctx, "RCLEUS-RAW", argv[0], (pointer(*)())RCLEUS, "");
  return 0;
} // pointer ___rcleus(context *ctx, int n, pointer *argv, pointer env)
