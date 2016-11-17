// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/ppl_attributes.h"
#include "vision_utils/ppl_tags_images.h"
#include "vision_utils/matrix_testing.h"

TEST(TestSuite, get_attribute) {
  people_msgs::Person pose;
  bool foo;
  EXPECT_FALSE(vision_utils::get_tag(pose, "foo", foo));
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void test_get_set_tag(_T value1, _T value2) {
  people_msgs::Person pose;
  _T value_read;
  EXPECT_FALSE(vision_utils::has_tag(pose, "attr1"));
  EXPECT_FALSE(vision_utils::get_tag(pose, "attr1", value_read));
  EXPECT_FALSE(vision_utils::has_tag(pose, "attr1"));

  // set attribute "attr1" to value1
  EXPECT_TRUE(vision_utils::set_tag(pose, "attr1", value1));
  EXPECT_TRUE(vision_utils::has_tag(pose, "attr1"));
  // check the attribute "attr1" is well equal to value1
  EXPECT_TRUE(vision_utils::get_tag(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value1) << "value_read:" << value_read << ", value1:" << value1;

  // set attribute "attr1" to value2
  EXPECT_TRUE(vision_utils::set_tag(pose, "attr1", value2));
  // check the attribute "attr1" is well equal to value2
  EXPECT_TRUE(vision_utils::get_tag(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value2) << "value_read:" << value_read << ", value2:" << value2;

  // set attribute "attr2" to value1
  EXPECT_TRUE(vision_utils::set_tag(pose, "attr2", value1));

      // check the attribute "attr2" is well equal to value1
  EXPECT_TRUE(vision_utils::get_tag(pose, "attr2", value_read));

  EXPECT_TRUE(value_read == value1) << "value_read:" << value_read << ", value1:" << value1;
  // check the attribute "attr1" is well equal to value2
  EXPECT_TRUE(vision_utils::get_tag(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value2) << "value_read:" << value_read << ", value2:" << value2;
}

TEST(TestSuite, get_set_tag) {
  test_get_set_tag<bool>(true, false);
  test_get_set_tag<short>(1, 2);
  test_get_set_tag<int>(1, 2);
  test_get_set_tag<float>(1, 2);
  test_get_set_tag<double>(1, 2);
  //const char* var_char_arr = "foo";
  //test_get_set_tag(var_char_arr);
  test_get_set_tag<std::string>("bar","zim");
}

TEST(TestSuite, copy_attribute) {
  people_msgs::Person src, dst;
  EXPECT_TRUE(vision_utils::set_tag(src, "i1", 1));
  EXPECT_TRUE(vision_utils::set_tag(src, "i2", 2));
  EXPECT_TRUE(vision_utils::set_tag(src, "f1", 1.5));
  EXPECT_TRUE(vision_utils::set_tag(src, "f2", 2.5));
  EXPECT_TRUE(vision_utils::set_tag(src, "s1", "string1"));
  EXPECT_TRUE(vision_utils::set_tag(src, "s2", "string2"));

  EXPECT_TRUE(vision_utils::set_tag(dst, "i2", 1));
  EXPECT_TRUE(vision_utils::set_tag(dst, "f2", 1.5));
  EXPECT_TRUE(vision_utils::set_tag(dst, "s2", "string1"));

  EXPECT_TRUE(vision_utils::copy_tags(src, dst));
  int i;
  EXPECT_TRUE(vision_utils::get_tag(dst, "i1", i));
  EXPECT_TRUE(i == 1);
  EXPECT_TRUE(vision_utils::get_tag(dst, "i2", i));
  EXPECT_TRUE(i == 2);
  double f;
  EXPECT_TRUE(vision_utils::get_tag(dst, "f1", f));
  EXPECT_TRUE(f == 1.5);
  EXPECT_TRUE(vision_utils::get_tag(dst, "f2", f));
  EXPECT_TRUE(f == 2.5);
  std::string s;
  EXPECT_TRUE(vision_utils::get_tag(dst, "s1", s));
  EXPECT_TRUE(s == "string1");
  EXPECT_TRUE(vision_utils::get_tag(dst, "s2", s));
  EXPECT_TRUE(s == "string2");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, image_tag) {
  cv::Mat3b rgb(100, 100), rgb2;
  cv::Mat1f depth(100, 100), depth2;
  cv::Mat1b user(100, 100), user2;

  people_msgs::Person src;
  // test for an empty image
  EXPECT_FALSE(vision_utils::get_image_tag(src, "foo", rgb2));

  // test for each type
  EXPECT_TRUE(vision_utils::set_image_tag(src, "rgb", rgb));
  EXPECT_TRUE(vision_utils::get_image_tag(src, "rgb", rgb2));
  EXPECT_TRUE(vision_utils::matrices_equal(rgb, rgb2));

  EXPECT_TRUE(vision_utils::set_image_tag(src, "depth", depth));
  EXPECT_TRUE(vision_utils::get_image_tag(src, "depth", depth2));
  EXPECT_TRUE(vision_utils::matrices_equal(depth, depth2));

  EXPECT_TRUE(vision_utils::set_image_tag(src, "user", user));
  EXPECT_TRUE(vision_utils::get_image_tag(src, "user", user2));
  EXPECT_TRUE(vision_utils::matrices_equal(user, user2));

  // change types
  EXPECT_TRUE(vision_utils::set_image_tag(src, "rgb", user));
  EXPECT_TRUE(vision_utils::get_image_tag(src, "rgb", user2));
  EXPECT_TRUE(vision_utils::matrices_equal(user, user2));

  // type cast
  EXPECT_TRUE(vision_utils::get_image_tag(src, "depth", user2));
  EXPECT_TRUE(vision_utils::matrices_equal(user, user2));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
