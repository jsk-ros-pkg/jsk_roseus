/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2026, JSK Lab, University of Tokyo.
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

// eustf.so for ROS 2, on top of tf2_ros.
//
// The same functions as eustf.cpp (ROS 1 tf), so eustf.l works on both.
// "transformer" and "transform listener" are tf2_ros::Buffer. The tf
// listener receives /tf and /tf_static in its own thread, so it does not
// need (ros::spin-once). The node of tf (roseus_tf_<pid>) is spun in a
// background thread for the tf2_ros::BufferClient, no callback calls euslisp.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <tuple>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/buffer_client.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#include "eus.h"

// the type of the functions given to defun() and add_module_initializer():
// pointer (*)() until EusLisp 9.31, pointer (*)(context *, int, pointer *)
// since 9.32 (for C23 / GCC 15). Take them from the declarations.
namespace eus_compat {
template <typename T> struct args;
template <typename R, typename... A> struct args<R(A...)> {
  template <size_t N> using type = typename std::tuple_element<N, std::tuple<A...>>::type;
};
}  // namespace eus_compat
typedef eus_compat::args<decltype(defun)>::type<3> eus_defun_fn_t;
typedef eus_compat::args<decltype(add_module_initializer)>::type<1> eus_init_fn_t;

extern "C" {
  pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env);
  void register_eustf(){
    char modname[] = "___eustf";
    return add_module_initializer(modname, (eus_init_fn_t)___eustf);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string
#undef min
#undef max

using std::string;

namespace eustf_ros2 {

rclcpp::Logger logger() { return rclcpp::get_logger("eustf"); }

// the node of tf, spun in a background thread
struct TfNode {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  ~TfNode() {
    if (executor) executor->cancel();
    if (thread.joinable()) thread.join();
  }
};
static std::unique_ptr<TfNode> s_tf_node;

rclcpp::Node::SharedPtr tf_node() {
  if (!rclcpp::ok()) {
    error(E_USER, "You must call (ros::roseus \"nodename\") before using tf");
  }
  if (!s_tf_node) {
    s_tf_node.reset(new TfNode());
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    s_tf_node->node = std::make_shared<rclcpp::Node>(
      "roseus_tf_" + std::to_string(getpid()), options);
    s_tf_node->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    s_tf_node->executor->add_node(s_tf_node->node);
    auto executor = s_tf_node->executor;
    s_tf_node->thread = std::thread([executor]() { executor->spin(); });
  }
  return s_tf_node->node;
}

struct Transformer {
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
};

Transformer * transformer(pointer p) {
  return (Transformer *)(intval(p));
}

// tf2 does not accept frame ids starting with /
string frame(pointer p) {
  if (!isstring(p)) error(E_NOSTRING);
  string s((char *)p->c.str.chars);
  while (!s.empty() && s[0] == '/') s = s.substr(1);
  return s;
}

tf2::TimePoint time_point(pointer p) {
  if (!(isvector(p) && elmtypeof(p) == ELM_INT)) error(E_NOVECTOR);
  int64_t sec = p->c.ivec.iv[0], nsec = p->c.ivec.iv[1];
  if (sec == 0 && nsec == 0) return tf2::TimePointZero;
  return tf2::TimePoint(std::chrono::nanoseconds(sec * 1000000000LL + nsec));
}

double number(pointer p) {
  numunion nu;
  if (isint(p)) return (double)intval(p);
  if (isflt(p)) return (double)fltval(p);
  error(E_NONUMBER);
  return 0;
}

pointer transform_to_eus(context *ctx, const geometry_msgs::msg::TransformStamped & trans) {
  pointer vs = makefvector(7);  //pos[3] + rot[4](angle-axis quaternion)
  vpush(vs);
  vs->c.fvec.fv[0] = trans.transform.translation.x;
  vs->c.fvec.fv[1] = trans.transform.translation.y;
  vs->c.fvec.fv[2] = trans.transform.translation.z;
  vs->c.fvec.fv[3] = trans.transform.rotation.w;
  vs->c.fvec.fv[4] = trans.transform.rotation.x;
  vs->c.fvec.fv[5] = trans.transform.rotation.y;
  vs->c.fvec.fv[6] = trans.transform.rotation.z;
  vpop();
  return vs;
}

/***********************************************************
 *   TF wrapper
 ************************************************************/

pointer EUSTF_TRANSFORMER(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  double cache_time = number(argv[1]);
  auto t = new Transformer();
  t->buffer = std::make_shared<tf2_ros::Buffer>(
    tf_node()->get_clock(), tf2::durationFromSec(cache_time > 0 ? cache_time : 10.0));
  return(makeint((eusinteger_t)t));
}

pointer EUSTF_ALLFRAMESASSTRING(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  string str = transformer(argv[0])->buffer->allFramesAsString();
  return(makestring((char *)(str.c_str()), str.length()));
}

pointer EUSTF_SETTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(7);
  Transformer *tf = transformer(argv[0]);
  if (!isvector(argv[1])) error(E_NOVECTOR);
  if (!isvector(argv[2])) error(E_NOVECTOR);
  eusfloat_t *pos = argv[1]->c.fvec.fv;
  eusfloat_t *rot = argv[2]->c.fvec.fv;
  if (!(isvector(argv[3]) && elmtypeof(argv[3]) == ELM_INT)) error(E_NOVECTOR);
  geometry_msgs::msg::TransformStamped transform;
  // pos (euslisp) -> [mm], pos (tf) -> [m]
  transform.transform.translation.x = pos[0]/1000.0;
  transform.transform.translation.y = pos[1]/1000.0;
  transform.transform.translation.z = pos[2]/1000.0;
  // rot (euslisp) -> (w, x, y, z)
  transform.transform.rotation.w = rot[0];
  transform.transform.rotation.x = rot[1];
  transform.transform.rotation.y = rot[2];
  transform.transform.rotation.z = rot[3];
  transform.header.frame_id = frame(argv[4]);
  transform.child_frame_id = frame(argv[5]);
  transform.header.stamp.sec = argv[3]->c.ivec.iv[0];
  transform.header.stamp.nanosec = argv[3]->c.ivec.iv[1];
  if (!isstring(argv[6])) error(E_NOSTRING);
  string authority((char*)(argv[6]->c.str.chars));
  bool ret = tf->buffer->setTransform(transform, authority);
  return(ret?T:NIL);
}

bool wait_for(std::function<bool()> can_transform, double timeout, double duration) {
  auto start = std::chrono::steady_clock::now();
  if (duration <= 0) duration = 0.01;
  while (rclcpp::ok()) {
    if (can_transform()) return true;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= timeout)
      return false;
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));
  }
  return false;
}

pointer EUSTF_WAITFORTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(6);
  Transformer *tf = transformer(argv[0]);
  string target_frame = frame(argv[1]);
  string source_frame = frame(argv[2]);
  tf2::TimePoint time = time_point(argv[3]);
  double timeout = number(argv[4]);
  double duration = number(argv[5]);
  string err_str;
  bool ret = wait_for([&]() {
      err_str.clear();
      return tf->buffer->canTransform(target_frame, source_frame, time, &err_str); },
    timeout, duration);
  if (!ret) RCLCPP_WARN_STREAM(logger(), "waitForTransform failed! : " << err_str);
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_WAITFORTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(8);
  Transformer *tf = transformer(argv[0]);
  string target_frame = frame(argv[1]);
  tf2::TimePoint target_time = time_point(argv[2]);
  string source_frame = frame(argv[3]);
  tf2::TimePoint source_time = time_point(argv[4]);
  string fixed_frame = frame(argv[5]);
  double timeout = number(argv[6]);
  double duration = number(argv[7]);
  string err_str;
  bool ret = wait_for([&]() {
      err_str.clear();
      return tf->buffer->canTransform(target_frame, target_time, source_frame, source_time,
                                      fixed_frame, &err_str); },
    timeout, duration);
  if (!ret) RCLCPP_WARN_STREAM(logger(), "waitForTransformFull failed! : " << err_str);
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  Transformer *tf = transformer(argv[0]);
  string err_str;
  bool ret = tf->buffer->canTransform(frame(argv[1]), frame(argv[2]), time_point(argv[3]), &err_str);
  if (!ret) RCLCPP_DEBUG_STREAM(logger(), "canTransform failed! : " << err_str);
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CANTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(7);
  Transformer *tf = transformer(argv[0]);
  string err_str;
  bool ret = tf->buffer->canTransform(frame(argv[1]), time_point(argv[2]),
                                      frame(argv[3]), time_point(argv[4]),
                                      frame(argv[5]), &err_str);
  if (!ret) RCLCPP_DEBUG_STREAM(logger(), "canTransformFull failed! : " << err_str);
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_CHAIN(register context *ctx,int n,pointer *argv)
{
  RCLCPP_ERROR(logger(), "%s is not implemented yet", __PRETTY_FUNCTION__);
  return(NIL);
}

pointer EUSTF_CLEAR(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  transformer(argv[0])->buffer->clear();
  return(T);
}

pointer EUSTF_FRAMEEXISTS(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  return(transformer(argv[0])->buffer->_frameExists(frame(argv[1]))?T:NIL);
}

pointer EUSTF_GETFRAMESTRINGS(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  std::vector<string> ids;
  transformer(argv[0])->buffer->_getFrameStrings(ids);
  pointer r = NIL;
  for (auto it = ids.rbegin(); it != ids.rend(); ++it) {
    vpush(r);
    pointer s = makestring((char *)(*it).c_str(), (*it).length());
    vpush(s);
    r = cons(ctx, s, r);
    vpop(); vpop();
  }
  return(r);
}

pointer EUSTF_GETLATERSTCOMMONTIME(register context *ctx,int n,pointer *argv)
{
  ckarg(3);
  Transformer *tf = transformer(argv[0]);
  string source_frame = frame(argv[1]);
  string target_frame = frame(argv[2]);
  try {
    auto trans = tf->buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return(cons(ctx, makeint(trans.header.stamp.sec), makeint(trans.header.stamp.nanosec)));
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR_STREAM(logger(), "getLatestCommonTime " << target_frame << " " << source_frame << " failed! : " << e.what());
    return(NIL);
  }
}

pointer EUSTF_LOOKUPTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg(4);
  Transformer *tf = transformer(argv[0]);
  try {
    auto trans = tf->buffer->lookupTransform(frame(argv[1]), frame(argv[2]), time_point(argv[3]));
    return transform_to_eus(ctx, trans);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
}

pointer EUSTF_LOOKUPTRANSFORMFULL(register context *ctx,int n,pointer *argv)
{
  ckarg(6);
  Transformer *tf = transformer(argv[0]);
  try {
    auto trans = tf->buffer->lookupTransform(frame(argv[1]), time_point(argv[2]),
                                             frame(argv[3]), time_point(argv[4]),
                                             frame(argv[5]));
    return transform_to_eus(ctx, trans);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
}

pointer EUSTF_TRANSFORMPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(5);  // tf, target_frame, time, frame_id, pose as float-vector (pos[3] + quaternion(w x y z))
  Transformer *tf = transformer(argv[0]);
  string target_frame = frame(argv[1]);
  if (!(isvector(argv[2]) && elmtypeof(argv[2]) == ELM_INT)) error(E_NOVECTOR);
  string frame_id = frame(argv[3]);
  if (!isvector(argv[4])) error(E_NOVECTOR);
  geometry_msgs::msg::PoseStamped input, output;
  input.pose.position.x = argv[4]->c.fvec.fv[0];
  input.pose.position.y = argv[4]->c.fvec.fv[1];
  input.pose.position.z = argv[4]->c.fvec.fv[2];
  input.pose.orientation.w = argv[4]->c.fvec.fv[3];
  input.pose.orientation.x = argv[4]->c.fvec.fv[4];
  input.pose.orientation.y = argv[4]->c.fvec.fv[5];
  input.pose.orientation.z = argv[4]->c.fvec.fv[6];
  input.header.stamp.sec = argv[2]->c.ivec.iv[0];
  input.header.stamp.nanosec = argv[2]->c.ivec.iv[1];
  input.header.frame_id = frame_id;
  try {
    output = tf->buffer->transform(input, target_frame);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
  pointer vs = makefvector(7);
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
  ckarg(5);
  Transformer *tf = transformer(argv[0]);
  string reference_frame = frame(argv[1]);
  string moving_frame = frame(argv[2]);
  double time = number(argv[3]);
  double duration = number(argv[4]);
  geometry_msgs::msg::VelocityStamped velocity;
  try {
    tf2::TimePoint tp = (time == 0) ? tf2::TimePointZero : tf2::timeFromSec(time);
    velocity = tf->buffer->lookupVelocity(moving_frame, reference_frame, tp,
                                          tf2::durationFromSec(duration));
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
  pointer vs = makefvector(6);
  vpush(vs);
  vs->c.fvec.fv[0] = velocity.velocity.linear.x;
  vs->c.fvec.fv[1] = velocity.velocity.linear.y;
  vs->c.fvec.fv[2] = velocity.velocity.linear.z;
  vs->c.fvec.fv[3] = velocity.velocity.angular.x;
  vs->c.fvec.fv[4] = velocity.velocity.angular.y;
  vs->c.fvec.fv[5] = velocity.velocity.angular.z;
  vpop();
  return(vs);
}

pointer EUSTF_TRANSFORM_LISTENER(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  double cache_time = number(argv[0]);
  auto node = tf_node();
  auto t = new Transformer();
  t->buffer = std::make_shared<tf2_ros::Buffer>(
    node->get_clock(), tf2::durationFromSec(cache_time > 0 ? cache_time : 10.0));
  t->buffer->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface()));
  // the listener spins its own thread
  t->listener = std::make_shared<tf2_ros::TransformListener>(*t->buffer, node, true);
  return(makeint((eusinteger_t)t));
}

pointer EUSTF_TRANSFORM_LISTENER_DISPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  delete transformer(argv[0]);
  return(T);
}

pointer EUSTF_SETEXTRAPOLATIONLIMIT(register context *ctx,int n,pointer *argv)
{
  RCLCPP_WARN(logger(), "set-extrapolation-limit is not supported by tf2");
  return(T);
}

pointer EUSTF_GETPARENT(register context *ctx,int n,pointer *argv)
{
  ckarg(3);
  Transformer *tf = transformer(argv[0]);
  string parent;
  try {
    bool ret = tf->buffer->_getParent(frame(argv[1]), time_point(argv[2]), parent);
    return(ret?makestring((char *)parent.c_str(),parent.length()):NIL);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
}

/* */
pointer EUSTF_TRANSFORM_BROADCASTER(register context *ctx,int n,pointer *argv)
{
  return(makeint((eusinteger_t)(new tf2_ros::TransformBroadcaster(tf_node()))));
}

pointer EUSTF_SEND_TRANSFORM(register context *ctx,int n,pointer *argv)
{
  /* ptr pos quarternion parent_frame_id, child_frame_id, time */
  ckarg(6);
  tf2_ros::TransformBroadcaster *bc = (tf2_ros::TransformBroadcaster *)(intval(argv[0]));
  if (!(isvector(argv[5]) && elmtypeof(argv[5]) == ELM_INT)) error(E_NOVECTOR);
  if (!isvector(argv[1]) || !isvector(argv[2])) error(E_NOVECTOR);
  eusfloat_t *pos = argv[1]->c.fvec.fv;
  eusfloat_t *quaternion = argv[2]->c.fvec.fv;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp.sec = argv[5]->c.ivec.iv[0];
  trans.header.stamp.nanosec = argv[5]->c.ivec.iv[1];
  trans.header.frame_id = frame(argv[3]);
  trans.child_frame_id = frame(argv[4]);
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

/* tf2 BufferClient */
pointer EUSTF_BUFFER_CLIENT(register context *ctx,int n,pointer *argv)
{
  /* &optional (ns "tf2_buffer_server") (check_frequency 10.0) (timeout_padding 2.0) */
  string ns_str("tf2_buffer_server");
  double check_frequency = 10.0;
  double timeout_padding = 2.0;
  ckarg2(0, 3);
  if (n > 0) {
    if (isstring(argv[0])) ns_str.assign((char *)(argv[0]->c.str.chars));
    else error(E_NOSTRING);
  }
  if (n > 1) check_frequency = number(argv[1]);
  if (n > 2) timeout_padding = number(argv[2]);
  return(makeint((eusinteger_t)(new tf2_ros::BufferClient(
    tf_node(), ns_str, check_frequency, tf2::durationFromSec(timeout_padding)))));
}

pointer EUSTF_BUFFER_CLIENT_DISPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  delete (tf2_ros::BufferClient *)(intval(argv[0]));
  return(T);
}

pointer EUSTF_TFBC_WAITFORSERVER(register context *ctx,int n,pointer *argv)
{
  ckarg2(1, 2);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  bool ret;
  if (n > 1) {
    ret = tfbc->waitForServer(tf2::durationFromSec(number(argv[1])));
  } else {
    ret = tfbc->waitForServer();
  }
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_TFBC_CANTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg2(4, 5);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  tf2::Duration timeout = tf2::durationFromSec(n > 4 ? number(argv[4]) : 0.0);
  string err_str;
  bool ret = false;
  try {
    ret = tfbc->canTransform(frame(argv[1]), frame(argv[2]), time_point(argv[3]), timeout, &err_str);
  } catch (const std::exception & e) {
    err_str = e.what();
  }
  if (!ret) RCLCPP_WARN_STREAM(logger(), "BufferClient::canTransform failed! : " << err_str);
  return((ret==true)?(T):(NIL));
}

pointer EUSTF_TFBC_LOOKUPTRANSFORM(register context *ctx,int n,pointer *argv)
{
  ckarg2(4, 5);
  tf2_ros::BufferClient *tfbc = (tf2_ros::BufferClient *)(intval(argv[0]));
  tf2::Duration timeout = tf2::durationFromSec(n > 4 ? number(argv[4]) : 0.0);
  try {
    auto trans = tfbc->lookupTransform(frame(argv[1]), frame(argv[2]), time_point(argv[3]), timeout);
    return transform_to_eus(ctx, trans);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "%s", e.what()); return(NIL);
  }
}

}  // namespace eustf_ros2

using namespace eustf_ros2;

#include "defun.h"
pointer ___eustf(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0) rospkg=makepkg(ctx,makestring("ROS", 3),NIL,NIL);
  Spevalof(PACKAGE)=rospkg;

  defun(ctx,"EUSTF-TRANSFORMER",argv[0],(eus_defun_fn_t)EUSTF_TRANSFORMER,"A Class which provides coordinate transforms between any two frames in a system");
  defun(ctx,"EUSTF-ALL-FRAMES-AS-STRING",argv[0],(eus_defun_fn_t)EUSTF_ALLFRAMESASSTRING,"A way to see what frames have been cached Useful for debugging");
  defun(ctx,"EUSTF-SET-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_SETTRANSFORM,"Add transform information to the tf data structure");
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_WAITFORTRANSFORM,"Block until a transform is possible or it times out");
  defun(ctx,"EUSTF-WAIT-FOR-TRANSFORM-FULL",argv[0],(eus_defun_fn_t)EUSTF_WAITFORTRANSFORMFULL,"Block until a transform is possible or it times out");
  defun(ctx,"EUSTF-CAN-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_CANTRANSFORM,"Test if a transform is possible");
  defun(ctx,"EUSTF-CAN-TRANSFORM-FULL",argv[0],(eus_defun_fn_t)EUSTF_CANTRANSFORMFULL,"Test if a transform is possible");
  defun(ctx,"EUSTF-CHAIN",argv[0],(eus_defun_fn_t)EUSTF_CHAIN,"Not implemented on ROS 2");
  defun(ctx,"EUSTF-CLEAR",argv[0],(eus_defun_fn_t)EUSTF_CLEAR,"Clear all data");
  defun(ctx,"EUSTF-FRAME-EXISTS",argv[0],(eus_defun_fn_t)EUSTF_FRAMEEXISTS,"Check if a frame exists in the tree");
  defun(ctx,"EUSTF-GET-FRAME-STRINGS",argv[0],(eus_defun_fn_t)EUSTF_GETFRAMESTRINGS,"A way to get a std::vector of available frame ids");
  defun(ctx,"EUSTF-GET-LATEST-COMMON-TIME",argv[0],(eus_defun_fn_t)EUSTF_GETLATERSTCOMMONTIME,"Return the latest rostime which is common across the spanning set zero if fails to cross");
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_LOOKUPTRANSFORM,"Get the transform between two frames by frame ID");
  defun(ctx,"EUSTF-LOOKUP-TRANSFORM-FULL",argv[0],(eus_defun_fn_t)EUSTF_LOOKUPTRANSFORMFULL,"Get the transform between two frames by frame ID");
  defun(ctx,"EUSTF-TRANSFORM-POSE",argv[0],(eus_defun_fn_t)EUSTF_TRANSFORMPOSE,"Transform a Stamped Pose into the target frame");
  defun(ctx,"EUSTF-LOOKUP-VELOCITY",argv[0],(eus_defun_fn_t)EUSTF_LOOKUPVELOCITY,"Lookup the twist of the tracking_frame with respect to the observation frame");
  defun(ctx,"EUSTF-TRANSFORM-LISTENER",argv[0],(eus_defun_fn_t)EUSTF_TRANSFORM_LISTENER,"tf2_ros::Buffer with tf2_ros::TransformListener");
  defun(ctx,"EUSTF-TRANSFORM-LISTENER-DISPOSE",argv[0],(eus_defun_fn_t)EUSTF_TRANSFORM_LISTENER_DISPOSE,"Destructor for TransformListener");
  defun(ctx,"EUSTF-SET-EXTRAPOLATION-LIMIT",argv[0],(eus_defun_fn_t)EUSTF_SETEXTRAPOLATIONLIMIT,"Not supported on tf2");
  defun(ctx,"EUSTF-GET-PARENT",argv[0],(eus_defun_fn_t)EUSTF_GETPARENT,"Fill the parent of a frame");
  defun(ctx,"EUSTF-TRANSFORM-BROADCASTER",argv[0],(eus_defun_fn_t)EUSTF_TRANSFORM_BROADCASTER,"tf2_ros::TransformBroadcaster");
  defun(ctx,"EUSTF-SEND-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_SEND_TRANSFORM,"Send a TransformStamped");
  defun(ctx,"EUSTF-BUFFER-CLIENT",argv[0],(eus_defun_fn_t)EUSTF_BUFFER_CLIENT,"Action client-based implementation of the tf2_ros::BufferInterface");
  defun(ctx,"EUSTF-BUFFER-CLIENT-DISPOSE",argv[0],(eus_defun_fn_t)EUSTF_BUFFER_CLIENT_DISPOSE,"tf2 BufferClient destructor");
  defun(ctx,"EUSTF-TF2-WAIT-FOR-SERVER",argv[0],(eus_defun_fn_t)EUSTF_TFBC_WAITFORSERVER,"Block until the action server is ready to respond to requests");
  defun(ctx,"EUSTF-TF2-CAN-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_TFBC_CANTRANSFORM,"Test if a transform is possible");
  defun(ctx,"EUSTF-TF2-LOOKUP-TRANSFORM",argv[0],(eus_defun_fn_t)EUSTF_TFBC_LOOKUPTRANSFORM,"Get the transform between two frames by frame ID");

  pointer_update(Spevalof(PACKAGE),p);
  return 0;
}
