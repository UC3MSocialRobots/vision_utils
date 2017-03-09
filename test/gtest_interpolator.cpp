/*!
  \file        gtest_interpolator.cpp
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

Some tests for interpolator functions

 */
// Bring in gtest
#include <gtest/gtest.h>
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
// vision_utils
#include "vision_utils/interpolator.h"

//! a quick function to build a standard ROS message
template<class _Msg>
_Msg factory(double data) {
  _Msg msg; msg.data = data; return msg;
}
//! a comparer function
template<class _Msg>
bool equals(const _Msg & msg, double data) {
  typename _Msg::_data_type data_cast = data;
  return msg.data == data_cast;
}
template<class _Msg>
bool between(const _Msg & msg, double min, double max) {
  typename _Msg::_data_type min_cast = min, max_cast = max;
  return msg.data > min_cast && msg.data < max_cast;
}
// template specifications
template<>
std_msgs::ColorRGBA factory(double data) {
  std_msgs::ColorRGBA msg;
  msg.r = msg.g = msg.b = msg.a = data;
  return msg;
}
template<>
bool equals(const std_msgs::ColorRGBA & msg, double data) {
  return msg.r == data && msg.g == data && msg.b == data && msg.a == data;
}
template<>
bool between(const std_msgs::ColorRGBA & msg, double min, double max) {
  return msg.r > min && msg.r < max && msg.g > min && msg.g < max
      && msg.b > min && msg.b < max && msg.a > min && msg.a < max;
}


TEST(TestSuite, ctor) {
  vision_utils::Interpolator<std_msgs::UInt8> inter;
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_empty() {
  ROS_INFO("test_empty()");
  vision_utils::Interpolator<_Msg> inter;
  std::vector<double> X;
  std::vector<_Msg> Y;
  ASSERT_FALSE(inter.train(X, Y));
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_singleton() {
  ROS_INFO("test_singleton()");
  vision_utils::Interpolator<_Msg> inter;
  std::vector<double> T(1, 42);
  std::vector<_Msg> X(1, factory<_Msg>(43));
  ASSERT_TRUE(inter.train(T, X));
  _Msg p;
  ASSERT_TRUE(inter.predict(42, p));
  ASSERT_TRUE(equals(p, 43));
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_pair() {
  ROS_INFO("test_pair()");
  vision_utils::Interpolator<_Msg> inter;
  std::vector<double> T;
  std::vector<_Msg> X;
  T.push_back(0);
  X.push_back(factory<_Msg>(10));
  T.push_back(1);
  X.push_back(factory<_Msg>(20));
  ASSERT_TRUE(inter.train(T, X));
  _Msg p;
  ASSERT_TRUE(inter.predict(0, p));
  ASSERT_TRUE(equals<_Msg>(p, 10));
  ASSERT_TRUE(inter.predict(1, p));
  ASSERT_TRUE(equals<_Msg>(p, 20));
  ASSERT_TRUE(inter.predict(0.5, p));
  ASSERT_TRUE(between<_Msg>(p, 10, 20));
}

////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
void test_suite() {
  test_empty<_Msg>();
  test_singleton<_Msg>();
  test_pair<_Msg>();
}

TEST(TestSuite, UInt8)   { test_suite<std_msgs::UInt8>(); }
TEST(TestSuite, Int8)    { test_suite<std_msgs::Int8>(); }
TEST(TestSuite, UInt16)  { test_suite<std_msgs::UInt16>(); }
TEST(TestSuite, Int16)   { test_suite<std_msgs::Int16>(); }
TEST(TestSuite, UInt32)  { test_suite<std_msgs::UInt32>(); }
TEST(TestSuite, Int32)   { test_suite<std_msgs::Int32>(); }
TEST(TestSuite, UInt64)  { test_suite<std_msgs::UInt64>(); }
TEST(TestSuite, Int64)   { test_suite<std_msgs::Int64>(); }
TEST(TestSuite, Float32) { test_suite<std_msgs::Float32>(); }
TEST(TestSuite, Float64) { test_suite<std_msgs::Float64>(); }
TEST(TestSuite, ColorRGBA) { test_suite<std_msgs::ColorRGBA>(); }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
