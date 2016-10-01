// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/ppl_attributes.h"

TEST(TestSuite, get_attribute) {
  people_msgs_rl::PeoplePose pose;
  bool foo;
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(pose, "foo", foo));
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void test_get_set_attribute(_T value1, _T value2) {
  people_msgs_rl::PeoplePose pose;
  _T value_read;
  EXPECT_FALSE(ppl_utils::has_attribute(pose, "attr1"));
  EXPECT_FALSE(ppl_utils::get_attribute_readonly(pose, "attr1", value_read));
  EXPECT_FALSE(ppl_utils::has_attribute(pose, "attr1"));

  // set attribute "attr1" to value1
  EXPECT_TRUE(ppl_utils::set_attribute(pose, "attr1", value1));
  EXPECT_TRUE(ppl_utils::has_attribute(pose, "attr1"));
  // check the attribute "attr1" is well equal to value1
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value1) << "value_read:" << value_read << ", value1:" << value1;

  // set attribute "attr1" to value2
  EXPECT_TRUE(ppl_utils::set_attribute(pose, "attr1", value2));
  // check the attribute "attr1" is well equal to value2
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value2) << "value_read:" << value_read << ", value2:" << value2;

  // set attribute "attr2" to value1
  EXPECT_TRUE(ppl_utils::set_attribute(pose, "attr2", value1));

      // check the attribute "attr2" is well equal to value1
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(pose, "attr2", value_read));

  EXPECT_TRUE(value_read == value1) << "value_read:" << value_read << ", value1:" << value1;
  // check the attribute "attr1" is well equal to value2
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(pose, "attr1", value_read));
  EXPECT_TRUE(value_read == value2) << "value_read:" << value_read << ", value2:" << value2;
}

TEST(TestSuite, get_set_attribute) {
  test_get_set_attribute<bool>(true, false);
  test_get_set_attribute<short>(1, 2);
  test_get_set_attribute<int>(1, 2);
  test_get_set_attribute<float>(1, 2);
  test_get_set_attribute<double>(1, 2);
  //const char* var_char_arr = "foo";
  //test_get_set_attribute(var_char_arr);
  test_get_set_attribute<std::string>("bar","zim");
}

TEST(TestSuite, copy_attribute) {
  people_msgs_rl::PeoplePose src, dst;
  EXPECT_TRUE(ppl_utils::set_attribute(src, "i1", 1));
  EXPECT_TRUE(ppl_utils::set_attribute(src, "i2", 2));
  EXPECT_TRUE(ppl_utils::set_attribute(src, "f1", 1.5));
  EXPECT_TRUE(ppl_utils::set_attribute(src, "f2", 2.5));
  EXPECT_TRUE(ppl_utils::set_attribute(src, "s1", "string1"));
  EXPECT_TRUE(ppl_utils::set_attribute(src, "s2", "string2"));

  EXPECT_TRUE(ppl_utils::set_attribute(dst, "i2", 1));
  EXPECT_TRUE(ppl_utils::set_attribute(dst, "f2", 1.5));
  EXPECT_TRUE(ppl_utils::set_attribute(dst, "s2", "string1"));

  EXPECT_TRUE(ppl_utils::copy_attributes(src, dst));
  int i;
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "i1", i));
  EXPECT_TRUE(i == 1);
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "i2", i));
  EXPECT_TRUE(i == 2);
  double f;
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "f1", f));
  EXPECT_TRUE(f == 1.5);
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "f2", f));
  EXPECT_TRUE(f == 2.5);
  std::string s;
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "s1", s));
  EXPECT_TRUE(s == "string1");
  EXPECT_TRUE(ppl_utils::get_attribute_readonly(dst, "s2", s));
  EXPECT_TRUE(s == "string2");
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
