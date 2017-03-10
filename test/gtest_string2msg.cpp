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
#include <ros/ros.h>
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
#include "vision_utils/iterable_to_string.h"

void test_empty() {
  std_msgs::Empty in, out_msg;
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in, out_str));
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
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
  ROS_INFO("out_str:'%s'", out_str.c_str());
}

TEST(TestSuite, string) {
  test_string("");
  test_string("OK");
  test_string("This is a sample of long sentence, it's cliché.");
  test_string("&é'(-è_çà)=^$ù*'\"");
}

////////////////////////////////////////////////////////////////////////////////

template<class _Type, class _Msg>
void test_dvec2msg(std::vector<double> & in_values) {
  // cast vector to message,
  _Msg in_msg, out_msg;
  ASSERT_TRUE(vision_utils::dvec2msg(in_values, in_msg));
  // then message to string,
  std::string out_str;
  ASSERT_TRUE(vision_utils::msg2string(in_msg, out_str));
  // then string back to message,
  ASSERT_TRUE(vision_utils::string2msg(out_str, out_msg));
  // then message then back to vector
  std::vector<double> out_values;
  ASSERT_TRUE(vision_utils::msg2dvec(out_msg, out_values));
  // we should have the same number of values
  unsigned int dim = in_values.size();
  ASSERT_TRUE(out_values.size() == dim)
      << "in_values:" << vision_utils::iterable_to_string(in_values)
      << "; out:" << vision_utils::iterable_to_string(out_values);
  // check all values are equal
  for (unsigned int i = 0; i < dim; ++i) {
    _Type in_value_cast = in_values[i];
    double error = fabs( in_value_cast - out_values[i]),
        max_error = 1E-3 * fabs(in_value_cast);
    ASSERT_TRUE(error <= max_error)
        << "in_values:" << vision_utils::iterable_to_string(in_values)
        << ", in_value_cast:" << in_value_cast
        << ", in_msg:" << in_msg
        << ", out_str:'" << out_str
        << "', out_msg:" << out_msg
        << ", out_values:" << vision_utils::iterable_to_string(out_values)
        << ", error:" << error
        << ", max_error:" << max_error;
  } // end for i
  ROS_INFO("out_str:'%s'", out_str.c_str());
}

template<class _Type, class _Msg>
void test_msg1(double x1) {
  std::vector<double> in_values;
  in_values.push_back(x1);
  test_dvec2msg<_Type, _Msg>(in_values);
}
template<class _Type, class _Msg>
void test_msg3(double x1, double x2, double x3) {
  std::vector<double> in_values;
  in_values.push_back(x1);
  in_values.push_back(x2);
  in_values.push_back(x3);
  test_dvec2msg<_Type, _Msg>(in_values);
}
template<class _Type, class _Msg>
void test_msg4(double x1, double x2, double x3, double x4) {
  std::vector<double> in_values;
  in_values.push_back(x1);
  in_values.push_back(x2);
  in_values.push_back(x3);
  in_values.push_back(x4);
  test_dvec2msg<_Type, _Msg>(in_values);
}
template<class _Type, class _Msg>
void test_msg6(double x1, double x2, double x3, double x4, double x5, double x6) {
  std::vector<double> in_values;
  in_values.push_back(x1);
  in_values.push_back(x2);
  in_values.push_back(x3);
  in_values.push_back(x4);
  in_values.push_back(x5);
  in_values.push_back(x6);
  test_dvec2msg<_Type, _Msg>(in_values);
}
template<class _Type, class _Msg>
void test_msg7(double x1, double x2, double x3, double x4, double x5,
               double x6, double x7) {
  std::vector<double> in_values;
  in_values.push_back(x1);
  in_values.push_back(x2);
  in_values.push_back(x3);
  in_values.push_back(x4);
  in_values.push_back(x5);
  in_values.push_back(x6);
  in_values.push_back(x7);
  test_dvec2msg<_Type, _Msg>(in_values);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class _Type, class _Msg>
void test_suite_msg1(bool skip_neg = false) {
  test_msg1<_Type, _Msg>(0);
  test_msg1<_Type, _Msg>(1);
  test_msg1<_Type, _Msg>(42);
  test_msg1<_Type, _Msg>(3.14);
  test_msg1<_Type, _Msg>(M_PI);
  test_msg1<_Type, _Msg>(123456789012345);
  test_msg1<_Type, _Msg>(5E20);
  if (skip_neg)
    return;
  test_msg1<_Type, _Msg>(-1);
  test_msg1<_Type, _Msg>(-5E20);
  test_msg1<_Type, _Msg>(-3.14);
  test_msg1<_Type, _Msg>(-5E-20);
}

// std_msgs::Bool actually embeds a "unsigned char" and not a bool
TEST(TestSuite, suite_Bool)    { test_suite_msg1<unsigned char, std_msgs::Bool>(); }
TEST(TestSuite, suite_UInt8)   { test_suite_msg1<unsigned char, std_msgs::UInt8>(true); }
TEST(TestSuite, suite_Int8)    { test_suite_msg1<char, std_msgs::Int8>(); }
TEST(TestSuite, suite_UInt16)  { test_suite_msg1<unsigned short, std_msgs::UInt16>(true); }
TEST(TestSuite, suite_Int16)   { test_suite_msg1<short, std_msgs::Int16>(); }
TEST(TestSuite, suite_UInt32)  { test_suite_msg1<unsigned int, std_msgs::UInt32>(true); }
TEST(TestSuite, suite_Int32)   { test_suite_msg1<int, std_msgs::Int32>(); }
TEST(TestSuite, suite_UInt64)  { test_suite_msg1<unsigned long, std_msgs::UInt64>(true); }
TEST(TestSuite, suite_Int64)   { test_suite_msg1<long, std_msgs::Int64>(); }
TEST(TestSuite, suite_Float32) { test_suite_msg1<float, std_msgs::Float32>(); }
TEST(TestSuite, suite_Float64) { test_suite_msg1<double, std_msgs::Float64>(); }

////////////////////////////////////////////////////////////////////////////////

template<class _Type, class _Msg>
void test_suite_msg3() {
  test_msg3<_Type, _Msg>(0,0,0);
  test_msg3<_Type, _Msg>(1,0,-1);
  test_msg3<_Type, _Msg>(.1,.2,.3);
  test_msg3<_Type, _Msg>(123456789012345,0,0);
  test_msg3<_Type, _Msg>(5E20,-5E20,5E-20);
}

TEST(TestSuite, Point32) { test_suite_msg3<float, geometry_msgs::Point32>(); }
TEST(TestSuite, Point) { test_suite_msg3<double, geometry_msgs::Point>(); }
TEST(TestSuite, Vector3) { test_suite_msg3<double, geometry_msgs::Vector3>(); }
TEST(TestSuite, Pose2D) { test_suite_msg3<double, geometry_msgs::Pose2D>(); }

////////////////////////////////////////////////////////////////////////////////

template<class _Type, class _Msg>
void test_suite_msg4() {
  test_msg4<_Type, _Msg>(0,0,0,0);
  test_msg4<_Type, _Msg>(1,0,-1,1);
  test_msg4<_Type, _Msg>(.1,.2,.3,.4);
  test_msg4<_Type, _Msg>(123456789012345,0,0,0);
  test_msg4<_Type, _Msg>(5E20,5E20,-5E20,0);
}

TEST(TestSuite, ColorRGBA) { test_suite_msg4<double, std_msgs::ColorRGBA>(); }
TEST(TestSuite, Quaternion) { test_suite_msg4<double, geometry_msgs::Quaternion>(); }

////////////////////////////////////////////////////////////////////////////////

template<class _Type, class _Msg>
void test_suite_msg6() {
  test_msg6<_Type, _Msg>(0,0,0, 0,0,0);
  test_msg6<_Type, _Msg>(1,0,-1, 1,0,-1);
  test_msg6<_Type, _Msg>(.1,.2,.3, .4,.5,.6);
  test_msg6<_Type, _Msg>(123456789012345,0,0,0,0,0);
  test_msg6<_Type, _Msg>(5E20,-5E20,5E-20,0,0,0);
}

TEST(TestSuite, Accel) { test_suite_msg6<double, geometry_msgs::Accel>(); }
TEST(TestSuite, Twist) { test_suite_msg6<double, geometry_msgs::Twist>(); }

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, Pose) {
  test_msg7<double, geometry_msgs::Pose>(0,0,0, 0,0,0, 1);
  test_msg7<double, geometry_msgs::Pose>(1,0,-1, 1,0,-1,1);
  test_msg7<double, geometry_msgs::Pose>(.1,.2,.3, .4,.5,.6,.7);
  test_msg7<double, geometry_msgs::Pose>(123456789012345,0,0,0,0,0,0);
  test_msg7<double, geometry_msgs::Pose>(5E20,-5E20,5E-20,0,0,0,0);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
