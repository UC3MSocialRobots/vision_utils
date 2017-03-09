/*!
  \file        gtest_string2msg.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/03/09
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Some tests for string2msg functions

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
// vision_utils
#include "vision_utils/string2msg.h"
#include "vision_utils/msg2string.h"

void test_empty() {
  std_msgs::Empty in, out_msg;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_single_number(double data) {
  _Msg in, out_msg;
  in.data = (typename _Msg::_data_type) data;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(fabs(in.data - out_msg.data) < 1E-3)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

template<class _Msg>
void test_batch() {
  test_single_number<_Msg>(0);
  test_single_number<_Msg>(1);
  test_single_number<_Msg>(-1);
  test_single_number<_Msg>(42);
  test_single_number<_Msg>(3.14);
  test_single_number<_Msg>(-3.14);
  test_single_number<_Msg>(M_PI);
}

TEST(TestSuite, batches_single_types) {
  test_batch<std_msgs::Bool>();
  test_batch<std_msgs::UInt8>();
  test_batch<std_msgs::Int8>();
  test_batch<std_msgs::UInt16>();
  test_batch<std_msgs::Int16>();
  test_batch<std_msgs::UInt32>();
  test_batch<std_msgs::Int32>();
  test_batch<std_msgs::UInt64>();
  test_batch<std_msgs::Int64>();
  test_batch<std_msgs::Float32>();
  test_batch<std_msgs::Float64>();
}

////////////////////////////////////////////////////////////////////////////////

void test_string(std::string data) {
  std_msgs::String in, out_msg;
  in.data = data;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.data == out_msg.data && in.data == data)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

TEST(TestSuite, string) {
  test_string("");
  test_string("OK");
  test_string("This is a sample of long sentence, it's cliché.");
  test_string("&é'(-è_çà)=^$ù*'\"");
}

////////////////////////////////////////////////////////////////////////////////

void test_rgba(double r, double g, double b, double a) {
  std_msgs::ColorRGBA in, out_msg;
  in.r = r;
  in.g = g;
  in.b = b;
  in.a = a;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.r == out_msg.r && in.g == out_msg.g
              && in.b == out_msg.b && in.a == out_msg.a)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

TEST(TestSuite, rgba) {
  test_rgba(0,0,0,0);
  test_rgba(1,0,1,0);
  test_rgba(.1,.2,.3,.4);
}


////////////////////////////////////////////////////////////////////////////////

void test_xyzw(double x, double y, double z, double w) {
  geometry_msgs::Quaternion in, out_msg;
  in.x = x;
  in.y = y;
  in.z = z;
  in.w = w;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.x == out_msg.x && in.y == out_msg.y
              && in.z == out_msg.z && in.w == out_msg.w)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

TEST(TestSuite, Quaternion) {
  test_xyzw(0,0,0,0);
  test_xyzw(1,0,1,0);
  test_xyzw(.1,.2,.3,.4);
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_xyz(double x, double y, double z) {
  _Msg in, out_msg;
  in.x = x; in.y = y; in.z = z;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.x == out_msg.x && in.y == out_msg.y && in.z == out_msg.z)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

template<class _Msg>
void test_xyz() {
  test_xyz<_Msg>(0,0,0);
  test_xyz<_Msg>(1,0,-1);
  test_xyz<_Msg>(.1,.2,.3);
}

TEST(TestSuite, point) { test_xyz<geometry_msgs::Point>(); }
TEST(TestSuite, point32) { test_xyz<geometry_msgs::Point32>(); }
TEST(TestSuite, Vector3) { test_xyz<geometry_msgs::Vector3>(); }

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_xyt(double x, double y, double theta) {
  _Msg in, out_msg;
  in.x = x; in.y = y; in.theta = theta;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.x == out_msg.x && in.y == out_msg.y && in.theta == out_msg.theta)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

template<class _Msg>
void test_xyt() {
  test_xyt<_Msg>(0,0,0);
  test_xyt<_Msg>(1,0,-1);
  test_xyt<_Msg>(.1,.2,.3);
}

TEST(TestSuite, Pose2D) { test_xyt<geometry_msgs::Pose2D>(); }

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_xyzxyz(double lx, double ly, double lz,
                 double ax, double ay, double az) {
  _Msg in, out_msg;
  in.linear.x = lx;  in.linear.y = ly;  in.linear.z = lz;
  in.angular.x = ax; in.angular.y = ay; in.angular.z = az;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.linear.x == out_msg.linear.x && in.linear.y == out_msg.linear.y && in.linear.z == out_msg.linear.z
              && in.angular.x == out_msg.angular.x
              && in.angular.y == out_msg.angular.y
              && in.angular.z == out_msg.angular.z)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

template<class _Msg>
void test_xyzxyz() {
  test_xyzxyz<_Msg>(0,0,0, 0,0,0);
  test_xyzxyz<_Msg>(1,0,-1, 1,0,-1);
  test_xyzxyz<_Msg>(.1,.2,.3, .4,.5,.6);
}

TEST(TestSuite, Accel) { test_xyzxyz<geometry_msgs::Accel>(); }
TEST(TestSuite, Twist) { test_xyzxyz<geometry_msgs::Twist>(); }

////////////////////////////////////////////////////////////////////////////////

void test_Pose(double px, double py, double pz,
                 double ox, double oy, double oz, double ow) {
  geometry_msgs::Pose in, out_msg;
  in.position.x = px;  in.position.y = py;  in.position.z = pz;
  in.orientation.x = ox; in.orientation.y = oy; in.orientation.z = oz;
  in.orientation.w = ow;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  ASSERT_TRUE(in.position.x == out_msg.position.x && in.position.y == out_msg.position.y && in.position.z == out_msg.position.z
              && in.orientation.x == out_msg.orientation.x
              && in.orientation.y == out_msg.orientation.y
              && in.orientation.z == out_msg.orientation.z
              && in.orientation.w == out_msg.orientation.w)
      << "in:" << in << ", out_str:" << out_str << ", out_msg:" << out_msg;
}

TEST(TestSuite, Pose) {
  test_Pose(0,0,0, 0,0,0, 1);
  test_Pose(1,0,-1, 1,0,-1,1);
  test_Pose(.1,.2,.3, .4,.5,.6,.7);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
