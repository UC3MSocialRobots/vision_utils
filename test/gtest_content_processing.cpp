/*!
  \file        gtest_content_processing.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/15

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

Some tests for \b MiniStage class
and \b cvstage_plugins namespace.

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <map>
#include <vision_utils/get_all_different_values.h>
#include <vision_utils/detect_end_points.h>
#include <vision_utils/img2string.h>
#include <vision_utils/get_all_non_null_values_and_com.h>
#include <vision_utils/img_path.h>
#include <vision_utils/iterable_to_int_string.h>
#include <vision_utils/iterable_to_string.h>
#include <vision_utils/map_keys_to_container.h>
#include <vision_utils/map_to_string.h>
#include <vision_utils/mask.h>
#include "vision_utils/matrix_testing.h"
#include <vision_utils/propagative_floodfill.h>
#include "vision_utils/region_growth.h"
#include "vision_utils/timer.h"
#include "vision_utils/user_image_to_rgb.h"

#define GRAYSCALE CV_LOAD_IMAGE_GRAYSCALE
bool display = false;

#if 1

////////////////////////////////////////////////////////////////////////////////

template<class _T>
_T make_struct(int min, int max) {
  _T ans;
  for (int i = min; i <= max; ++i)
    ans.insert(ans.end(), (uchar) i);
  return ans;
}

TEST(TestSuite, get_all_different_values) {
  cv::Mat1b query = cv::imread(vision_utils::IMG_DIR() + "balloon_masks.png", GRAYSCALE);

  std::set<uchar> out_set, correct_set = make_struct<std::set<uchar> >(0, 3);
  vision_utils::Timer timer;
  vision_utils::get_all_different_values(query, out_set, false);
  timer.printTime("get_all_different_values(set with zeros)");
  ASSERT_TRUE(out_set == correct_set) << "out_set:" << vision_utils::iterable_to_string(out_set);

  correct_set = make_struct<std::set<uchar> >(1, 3);
  timer.reset();
  vision_utils::get_all_different_values(query, out_set, true);
  timer.printTime("get_all_different_values(set without zeros)");
  ASSERT_TRUE(out_set == correct_set) << "out_set:" << vision_utils::iterable_to_string(out_set);

  std::vector<uchar> out_vec, correct_vec = make_struct<std::vector<uchar> >(0, 3);
  vision_utils::get_all_different_values(query, out_vec, false);
  timer.printTime("get_all_different_values(set with zeros)");
  ASSERT_TRUE(out_vec == correct_vec) << "out_vec:" << vision_utils::iterable_to_string(out_vec);

  correct_vec = make_struct<std::vector<uchar> >(1, 3);
  timer.reset();
  vision_utils::get_all_different_values(query, out_vec, true);
  timer.printTime("get_all_different_values(vec without zeros)");
  ASSERT_TRUE(out_vec == correct_vec) << "out_vec:" << vision_utils::iterable_to_string(out_vec);
}


////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_all_different_values2) {
  cv::Mat1b query = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_user_mask.png", GRAYSCALE);
  std::set<uchar> out_set, correct_set = make_struct<std::set<uchar> >(0, 2);
  vision_utils::Timer timer;
  vision_utils::get_all_different_values(query, out_set, false);
  timer.printTime("get_all_different_values(set with zeros)");
  ASSERT_TRUE(out_set == correct_set) << "out_set:" << vision_utils::iterable_to_string(out_set);

  correct_set = make_struct<std::set<uchar> >(1, 2);
  timer.reset();
  vision_utils::get_all_different_values(query, out_set, true);
  timer.printTime("get_all_different_values(set without zeros)");
  ASSERT_TRUE(out_set == correct_set) << "out_set:" << vision_utils::iterable_to_string(out_set);
}


////////////////////////////////////////////////////////////////////////////////
void test_get_all_different_values_and_com(const cv::Mat1b & query,
                                           unsigned int minblobidx, unsigned int maxblobidx){
  printf("\n");
  std::map<uchar, cv::Point> out_map;
  for (unsigned int use_fast = 0; use_fast <= 1; ++use_fast) {
    for (unsigned int ignore_zeros = 0; ignore_zeros <= 1; ++ignore_zeros) {
      for (unsigned int ensure_com_on_mask = 0; ensure_com_on_mask <= 1; ++ensure_com_on_mask) {
        vision_utils::Timer timer;
        if (use_fast)
          vision_utils::get_all_non_null_values_and_com_fast
              (query, out_map, ignore_zeros, ensure_com_on_mask);
        else
          vision_utils::get_all_non_null_values_and_com
              (query, out_map, ignore_zeros, ensure_com_on_mask);
        printf("Time for get_all_different_values_and_com"
               "(use_fast:%i, ignore_zeros:%i, ensure_com_on_mask:%i): %g ms\n",
               use_fast, ignore_zeros, ensure_com_on_mask, timer.time());

        ASSERT_TRUE(out_map.size() == maxblobidx - minblobidx+ (ignore_zeros ? 1: 2))
            << "out_map:" << vision_utils::map_to_string(out_map);
        std::vector<uchar> out_keys,
            correct_keys = make_struct<std::vector<uchar> >(minblobidx, maxblobidx);
        if (!ignore_zeros)
          correct_keys.insert(correct_keys.begin(), (uchar) 0);
        vision_utils::map_keys_to_container(out_map, out_keys);
        ASSERT_TRUE(out_keys == correct_keys)
            << "out_keys:" << vision_utils::iterable_to_int_string(out_keys)
            << "correct_keys:" << vision_utils::iterable_to_int_string(correct_keys);

        // check the COM are on the masks
        std::map<uchar, cv::Point>::const_iterator out_it = out_map.begin();
        if (ensure_com_on_mask) {
          while (out_it != out_map.end()) {
            ASSERT_TRUE(query(out_it->second) == out_it->first);
            ++out_it;
          } // end while (out_it != out.end)
        } // end if (ensure_com_on_mask)

        // illus
#if 0
        out_it = out_map.begin();
        cv::Mat3b illus;  user_image_to_rgb(query, illus);
        while (out_it != out_map.end()) {
          cv::circle(illus, out_it->second, 2, CV_RGB(255, 255, 255), -1);
          ++out_it;
        } // end while (out_it != out.end)
        cv::imshow("illus", illus);
        cv::waitKey(0);
#endif
      } // end loop use_fast
    } // end loop ensure_com_on_mask
  } // end loop ignore_zeros
} // end test_get_all_different_values_and_com();

TEST(TestSuite, get_all_different_values_and_com) {
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "balloon_masks.png", GRAYSCALE), 1, 3);
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_user_mask.png", GRAYSCALE), 1, 2);
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud2_user_mask.png", GRAYSCALE), 1, 2);
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud3_user_mask.png", GRAYSCALE), 1, 2);
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "depth/alberto1_user_mask.png", GRAYSCALE), 255, 255);
  test_get_all_different_values_and_com
      (cv::imread(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", GRAYSCALE) / 255, 1, 1);
  test_get_all_different_values_and_com // no normalization
      (cv::imread(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", GRAYSCALE), 255, 255);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, RegionGrower_empty_img) {
  vision_utils::RegionGrower<uchar> rg;
  std::vector<cv::Point> pts;
  bool ok = rg.grow(cv::Mat1b(), cv::Point(), 1, 1, pts);
  ASSERT_TRUE(ok == false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, RegionGrower_rectangle) {
  int cols = 120, rw = 40, rh = 80, o = 10;
  cv::Mat1b img(cols, cols, (uchar) 0);
  cv::rectangle(img, cv::Point(o, o), cv::Point(o+rw, o+rh), cv::Scalar::all(255), -1);
  cv::rectangle(img, cv::Point(o+rw, o), cv::Point(o+2*rw, o+rh), cv::Scalar::all(200), -1);
  // cv::imshow("img", img); cv::waitKey(0);
  vision_utils::RegionGrower<uchar> rg;
  std::vector<cv::Point> pts;
  // std::cout << "img:" << std::endl << img << std::endl;
  bool ok;

  unsigned int ntimes = 10;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time)
    ok = rg.grow(img, cv::Point(o + 2, o + 2), 1, 1, pts);
  timer.printTime_factor("rg.grow()", ntimes);
  ASSERT_TRUE(ok == true);
  ASSERT_TRUE((int) pts.size() == (rh+1) * rw)
      << "expected:" << (rh+1) * rw << ", pts:" << pts.size();

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    ok = rg.grow(img, cv::Point(o + 2, o + 2), 100, 100, pts);
  timer.printTime_factor("rg.grow()", ntimes);
  ASSERT_TRUE(ok == true);
  ASSERT_TRUE((int) pts.size() == (rh+1) * (1+2*rw))
      << "expected:" << (rh+1) * (1+2*rw) << ", pts:" << pts.size();
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, propagative_floodfill_line) {
  cv::Mat1b img(50, 50);
  // N E S W
  for (unsigned int direction = 0; direction < 4; ++direction) {
    // prepare image
    img.setTo(0);
    cv::Point center(img.cols /2, img.rows / 2), end = center;
    int length = 15;
    end.x += (direction == 1 ? length : (direction == 3) ? - length : 0);
    end.y += (direction == 2 ? length : (direction == 0) ? - length : 0);
    cv::line(img, center, end, cv::Scalar::all(255), 1);
    //cv::imshow("img", img); cv::waitKey(0);
    ASSERT_TRUE(cv::countNonZero(img) == length + 1)
        << "cv::countNonZero(img):" << cv::countNonZero(img);

    // floodfill
    SEEN_BUFFER_TYPE seen_buffer_short;
    vision_utils::propagative_floodfill(img, center, seen_buffer_short);

    // check it is correct
    ASSERT_TRUE(cv::countNonZero(seen_buffer_short) == length + 1)
        << "cv::countNonZero(seen_buffer_short):" << cv::countNonZero(seen_buffer_short);

    ASSERT_TRUE((int) seen_buffer_short.at<short>(center) == 1)
        << "seen_buffer_short.at<short>(center):" << seen_buffer_short.at<short>(center);
    ASSERT_TRUE((int) seen_buffer_short.at<short>(end) == length + 1)
        << "seen_buffer_short.at<short>(end):" << seen_buffer_short.at<short>(end);
  } // end for (direction)
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, propagative_floodfill_circle) {
  unsigned int rows = 50, cols = 20;
  cv::Mat1b img(rows, cols);
  // prepare image
  cv::Point center(cols /2, rows / 2);
  //int radius = 15;
  // img.setTo(0);
  //cv::circle(img, center, radius, cv::Scalar::all(255), -1);
  img.setTo(255);

  // floodfill without diagonals
  SEEN_BUFFER_TYPE seen_buffer_short;
  unsigned int ntimes = 10;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time) {
    vision_utils::propagative_floodfill(img, center, seen_buffer_short,
                                       false, NULL, NULL, NULL, false);
  } // end for (time)
  timer.printTime_factor("propagative_floodfill()", ntimes);


  // display buffer
  //  cv::Mat1f seen_buffer_float_buffer;
  //  cv::Mat3b seen_buffer_illus;
  //  vision_utils::propagative_floodfill_seen_buffer_to_viz_image
  //      (seen_buffer_short, seen_buffer_float_buffer, seen_buffer_illus);
  //  cv::imshow("seen_buffer_illus", seen_buffer_illus); cv::waitKey(0);

  for (unsigned int row = 0; row < rows; ++row) {
    // get the address of row
    short* data = seen_buffer_short.ptr<short>(row);
    for (unsigned int col = 0; col < cols; ++col) {
      int l1dist = abs(col - center.x) + abs(row - center.y);
      ASSERT_TRUE(data[col] == 1 + l1dist)
          << "(" << col << "," << row
          << "): expected:" << 1 + l1dist << ", obtained:" << data[col];
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, PropagativeFloodfiller_line) {
  cv::Mat1b img(50, 50);
  // N E S W
  for (unsigned int direction = 0; direction < 4; ++direction) {
    // prepare image
    img.setTo(0);
    cv::Point center(img.cols /2, img.rows / 2), end = center;
    int length = 15;
    end.x += (direction == 1 ? length : (direction == 3) ? - length : 0);
    end.y += (direction == 2 ? length : (direction == 0) ? - length : 0);
    cv::line(img, center, end, cv::Scalar::all(255), 1);
    //cv::imshow("img", img); cv::waitKey(0);
    ASSERT_TRUE(cv::countNonZero(img) == length + 1)
        << "cv::countNonZero(img):" << cv::countNonZero(img);

    // floodfill
    vision_utils::PropagativeFloodfiller<uchar> pff;
    pff.floodfill(img, center, false, true);

    // check it is correct
    ASSERT_TRUE(pff.get_seen_pts_nb() == length + 1)
        << "pff.get_seen_pts_nb():" << pff.get_seen_pts_nb();

    ASSERT_TRUE(pff.value_at(center) == 1)
        << "pff.value_at(center):" << pff.value_at(center);
    ASSERT_TRUE((int) pff.value_at(end) == 1+length)
        << "expected:" << 1+length << ", pff.value_at(end):" << pff.value_at(end);

    // now check the render as an image
    cv::Mat3b illus_img;
    pff.illus_img(illus_img);
    cv::Mat1b illus_img_bw; // illus_img non null values should be = illus
    vision_utils::mask(illus_img, illus_img_bw, vision_utils::is_zero_vec3b);
    //cv::imshow("illus_img", illus_img); cv::imshow("illus_img_bw", illus_img_bw);  cv::waitKey(0);
    //cv::imwrite("img.png", img); cv::imwrite("illus_img_bw.png", illus_img_bw);
    illus_img_bw(center) = 255; // dirty fix
    ASSERT_TRUE(vision_utils::matrices_equal(illus_img_bw, img));
  } // end for (direction)
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, PropagativeFloodfiller_circle) {
  unsigned int rows = 50, cols = 20;
  cv::Mat1b img(rows, cols);
  // prepare image
  cv::Point center(cols /2, rows / 2);
  //int radius = 15;
  // img.setTo(0);
  //cv::circle(img, center, radius, cv::Scalar::all(255), -1);
  img.setTo(255);

  // floodfill without diagonals
  vision_utils::PropagativeFloodfiller<uchar> pff;
  unsigned int ntimes = 10;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time) {
    pff.floodfill(img, center, false, false);
  } // end for (time)
  timer.printTime_factor("pff.floodfill()", ntimes);
  printf("seen_pts:%i\n", pff.get_seen_pts_nb());

  // display buffer
  //  cv::Mat1f seen_buffer_float_buffer;
  //  cv::Mat3b seen_buffer_illus;
  //  vision_utils::PropagativeFloodfiller_seen_buffer_to_viz_image
  //      (seen_buffer_short, seen_buffer_float_buffer, seen_buffer_illus);
  //  cv::imshow("seen_buffer_illus", seen_buffer_illus); cv::waitKey(0);

  for (unsigned int row = 0; row < rows; ++row) {
    // get the address of row
    for (unsigned int col = 0; col < cols; ++col) {
      int l1dist = abs(col - center.x) + abs(row - center.y);
      cv::Point p(col, row);
      ASSERT_TRUE(pff.value_at(p) == 1+l1dist)
          << p << ": expected:" << 1+l1dist << ", obtained:" << pff.value_at(p);
    } // end loop col
  } // end loop row

  // now check the render as an image
  cv::Mat3b illus_img;
  pff.illus_img(illus_img);
  cv::Mat1b illus_img_bw; // illus_img non null values should be = illus
  vision_utils::mask(illus_img, illus_img_bw, vision_utils::is_zero_vec3b);
  // cv::imshow("illus_img", illus_img); cv::imshow("illus_img_bw", illus_img_bw);  cv::waitKey(0);
  ASSERT_TRUE(vision_utils::matrices_equal(illus_img_bw, img));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask_empty_img) {
  vision_utils::ClosestPointInMask cl;
  cv::Point ans = cl.find(cv::Mat1b(), cv::Point());
  ASSERT_TRUE(ans.x < 0 && ans.y < 0) << "ans:" << ans;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask_empty_full) {
  vision_utils::ClosestPointInMask cl;
  cv::Mat1b mask(10, 10, (uchar) 0);
  cv::Point seed(5, 5);
  cv::Point ans = cl.find(mask, seed);
  ASSERT_TRUE(ans.x < 0 && ans.y < 0) << "ans:" << ans;

  mask.setTo(255);
  ans = cl.find(mask, seed);
  ASSERT_TRUE(ans == seed) << "ans:" << ans << ", seed:" << seed;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask_single_point) {
  vision_utils::ClosestPointInMask cl;
  int cols = 10;
  cv::Mat1b mask(cols, cols);
  for (unsigned int time = 0; time < 10; ++time) {
    cv::Point seed(rand() % cols, rand() % cols), peak(rand() % cols, rand() % cols);
    mask.setTo(0);
    mask(peak) = 255;
    cv::Point ans = cl.find(mask, seed, (uchar) 255, (uchar) 255);
    ASSERT_TRUE(ans == peak) << "ans:" << ans << ", peak:" << seed;
  } // end loop time
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask_david_arnaud1) {
  unsigned int ntimes = 100;
  vision_utils::ClosestPointInMask cl;
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_user_mask.png", GRAYSCALE);
  int cols = mask.cols, rows = mask.rows;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time) {
    cv::Point seed(rand() % cols, rand() % rows);
    cv::Point ans1 =
        cl.find(mask, seed, (uchar) 1, (uchar) 1);
    cv::Point ans2 =
        cl.find(mask, seed, (uchar) 2, (uchar) 2);
    // illus
    ASSERT_TRUE(vision_utils::bbox_full(mask).contains(ans1)) << "ans1:" << ans1;
    ASSERT_TRUE(vision_utils::bbox_full(mask).contains(ans2)) << "ans2:" << ans2;
    ASSERT_TRUE(mask(ans1) == (uchar) 1);
    ASSERT_TRUE(mask(ans2) == (uchar) 2);
    //  cv::Mat3b illus;  user_image_to_rgb(mask, illus); illus /= 2;
    //  cv::line(illus, seed, ans1, CV_RGB(255, 255, 0), 1);
    //  cv::line(illus, seed, ans2, CV_RGB(255, 255, 0), 1);
    //  cv::imshow("illus", illus); cv::waitKey(0);
  } // end loop time
  timer.printTime_factor("cl.find() for both users", ntimes);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask2_empty_img) {
  vision_utils::ClosestPointInMask2<uchar> cl;
  cv::Point ans = cl.find(cv::Mat1b(), cv::Point());
  ASSERT_TRUE(ans.x < 0 && ans.y < 0) << "ans:" << ans;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask2_empty_full) {
  vision_utils::ClosestPointInMask2<uchar> cl;
  cv::Mat1b mask(10, 10, (uchar) 0);
  cv::Point seed(5, 5);
  cv::Point ans = cl.find(mask, seed);
  ASSERT_TRUE(ans.x < 0 && ans.y < 0) << "ans:" << ans;

  mask.setTo(255);
  ans = cl.find(mask, seed);
  ASSERT_TRUE(ans == seed) << "ans:" << ans << ", seed:" << seed;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask2_single_point) {
  vision_utils::ClosestPointInMask2<uchar> cl;
  int cols = 10;
  cv::Mat1b mask(cols, cols);
  for (unsigned int time = 0; time < 10; ++time) {
    cv::Point seed(rand() % cols, rand() % cols), peak(rand() % cols, rand() % cols);
    mask.setTo(0);
    mask(peak) = 255;
    cv::Point ans = cl.find(mask, seed, (uchar) 255, (uchar) 255);
    ASSERT_TRUE(ans == peak) << "ans:" << ans << ", peak:" << seed;
  } // end loop time
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ClosestPointInMask2_david_arnaud1) {
  unsigned int ntimes = 100;
  vision_utils::ClosestPointInMask2<uchar> cl;
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_user_mask.png", GRAYSCALE);
  int cols = mask.cols, rows = mask.rows;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time) {
    cv::Point seed(rand() % cols, rand() % rows);
    cv::Point ans1 =
        cl.find(mask, seed, (uchar) 1, (uchar) 1);
    cv::Point ans2 =
        cl.find(mask, seed, (uchar) 2, (uchar) 2);
    // illus
    ASSERT_TRUE(vision_utils::bbox_full(mask).contains(ans1)) << "ans1:" << ans1;
    ASSERT_TRUE(vision_utils::bbox_full(mask).contains(ans2)) << "ans2:" << ans2;
    ASSERT_TRUE(mask(ans1) == (uchar) 1);
    ASSERT_TRUE(mask(ans2) == (uchar) 2);
    //  cv::Mat3b illus;  user_image_to_rgb(mask, illus); illus /= 2;
    //  cv::line(illus, seed, ans1, CV_RGB(255, 255, 0), 1);
    //  cv::line(illus, seed, ans2, CV_RGB(255, 255, 0), 1);
    //  cv::imshow("illus", illus); cv::waitKey(0);
  } // end loop time
  timer.printTime_factor("cl.find() for both users", ntimes);
  printf("seen_pts:%i\n", cl.get_seen_pts_nb());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ShortestPathFinder_empty_img) {
  vision_utils::ShortestPathFinder<uchar> spf;
  std::vector<cv::Point> pts;
  bool ok = spf.find(cv::Mat1b(), cv::Point(), cv::Point(), pts);
  ASSERT_TRUE(ok == false);
}

////////////////////////////////////////////////////////////////////////////////

void test_ShortestPathFinder(const cv::Mat1b & in,
                             const cv::Point & begin,
                             const cv::Point & end,
                             const cv::Mat1b & correct_ans,
                             bool is_correct_ans_wider = false) {
  vision_utils::ShortestPathFinder<uchar> spf;
  std::vector<cv::Point> pts;
  bool ok;
  unsigned int ntimes = 10;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time)
    ok = spf.find(in, begin, end, pts);
  timer.printTime_factor("spf.find()", ntimes);
  // std::cout << "pts:" << vision_utils::iterable_to_string(pts) << std::endl;
  ASSERT_TRUE(ok == true);
  ASSERT_TRUE(pts.front() == begin) << "pts:" << vision_utils::iterable_to_string(pts);
  ASSERT_TRUE(pts.back() == end) << "pts:" << vision_utils::iterable_to_string(pts);

  // check pts = in (only one path)
  cv::Mat1b out(in.size(), (uchar) 0);
  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx)
    out(pts[pt_idx]) = (uchar) 255;
  // cv::line(out, pts[pt_idx], pts[pt_idx], cv::Scalar::all(255));
  // cv::imshow("out", out); cv::waitKey(0);
  if (!correct_ans.empty()) {
    cv::Mat1b correct_ans_thin = correct_ans.clone();
    if (is_correct_ans_wider)
      correct_ans_thin = (correct_ans == out) & (correct_ans != 0);
    // cv::imshow("out", out);
    // cv::imshow("correct_ans", correct_ans);
    // cv::imshow("correct_ans_thin", correct_ans_thin);
    // cv::waitKey(0);
    // cv::imwrite("out.png", out); cv::imwrite("correct_ans.png", correct_ans); cv::imwrite("correct_ans_thin.png", correct_ans_thin);
    // std::cout << "in:" << in << std::endl;
    // std::cout << "out:" << out << std::endl;
    // std::cout << "correct_ans:" << correct_ans << std::endl;
    // std::cout << "correct_ans_thin:" << correct_ans_thin << std::endl;
    ASSERT_TRUE(vision_utils::matrices_equal(correct_ans_thin, out));
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ShortestPathFinder_simple_line) {
  int cols = 10;
  cv::Mat1b in(cols, cols, (uchar) 0);
  cv::Point begin(2, 5), end(8, 5);
  cv::line(in, begin, end, cv::Scalar::all(255));
  test_ShortestPathFinder(in, begin, end, in);
}

TEST(TestSuite, ShortestPathFinder_thick_line) {
  int cols = 10;
  cv::Mat1b in(cols, cols, (uchar) 0), correct_ans = in.clone();
  cv::Point begin(2, 5), end(8, 5);
  cv::line(in, begin, end, cv::Scalar::all(255), 2);
  cv::line(correct_ans, begin, end, cv::Scalar::all(255), 1);
  test_ShortestPathFinder(in, begin, end, correct_ans);
}

TEST(TestSuite, ShortestPathFinder_skeleton) {
  cv::Mat1b in = (cv::imread(vision_utils::IMG_DIR() + "skeletons/ref_skel.png", GRAYSCALE) == 255);
  cv::Point begin(127, 36), end(59, 393);
  test_ShortestPathFinder(in, begin, end, cv::Mat());
}

void test_maze(const std::string & filename, cv::Point begin, cv::Point end) {
  cv::Mat1b maze = cv::imread(filename, GRAYSCALE);
  cv::Mat1b in = (maze != 0), correct_ans = (maze == 255);
  test_ShortestPathFinder(in, begin, end, correct_ans, true);
}

TEST(TestSuite, ShortestPathFinder_maze) {
  test_maze(vision_utils::IMG_DIR() + "maze_solved.png", cv::Point(22, 124), cv::Point(173, 25));
}

TEST(TestSuite, ShortestPathFinder_maze_circular) {
  test_maze(vision_utils::IMG_DIR() + "maze-circular.png", cv::Point(459, 5), cv::Point(431, 434));
}

TEST(TestSuite, ShortestPathFinder_maze_harder) {
  test_maze(vision_utils::IMG_DIR() + "maze-harder-solved.png", cv::Point(16, 6), cv::Point(1985, 2017));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool pt_sorter (cv::Point A, cv::Point B) { return (A.x < B.x) || (A.x == B.x && A.y <= B.y); }

void test_EndFinder(const cv::Mat1b & mask,
                    const std::vector<cv::Point> & correct_end_pts,
                    bool print_time = true) {
  std::vector<cv::Point> end_pts;
  vision_utils::Timer timer;
  bool ok = vision_utils::detect_end_points(mask, end_pts);
  if (print_time)
    timer.printTime("vision_utils::detect_end_points()");
  ASSERT_TRUE(ok);

  std::vector<cv::Point> end_pts_sorted = end_pts, correct_end_pts_sorted = correct_end_pts;
  std::sort(end_pts_sorted.begin(), end_pts_sorted.end(), pt_sorter);
  std::sort(correct_end_pts_sorted.begin(), correct_end_pts_sorted.end(), pt_sorter);
  ASSERT_TRUE(end_pts_sorted == correct_end_pts_sorted)
      << "mask:" << vision_utils::img2string(mask)
         //<< ", has_no_neigh2string:" << finder.has_no_neigh2string()
      << ", correct_end_pts_sorted:" << vision_utils::iterable_to_string(correct_end_pts_sorted)
      << ", end_pts_sorted:" << vision_utils::iterable_to_string(end_pts_sorted);
}

TEST(TestSuite, EndFinder_empty_img) {
  std::vector<cv::Point> correct_end_pts;
  test_EndFinder(cv::Mat1b(), correct_end_pts);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, EndFinder_black_img) {
  std::vector<cv::Point> correct_end_pts;
  test_EndFinder(cv::Mat1b(50, 50, (uchar) 0), correct_end_pts);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, EndFinder_single) {
  unsigned int ncols = 10, nrows = 10;
  cv::Mat1b mask(nrows, ncols, (uchar) 0);
  for (unsigned int row = 0; row < nrows; ++row) {
    for (unsigned int col = 0; col < ncols; ++col) {
      // printf("\n EndFinder_single(%i, %i)\n", col, row);
      cv::Point single(col, row);
      mask.setTo(0);
      mask(single) = 255;
      test_EndFinder(mask, std::vector<cv::Point>(1, single), (col + row == 0));
    } // end loop col
  } // end loop row
}


////////////////////////////////////////////////////////////////////////////////

void test_EndFinder_cross(int padding) {
  int cols = 10, rows = 10;
  cv::Mat1b mask(rows, cols, (uchar) 0);
  cv::Point A1(padding, rows/ 2), A2(cols-1 - padding, rows/ 2),
      B1(cols / 2, padding), B2(cols / 2, rows-1 - padding);
  cv::line(mask, A1, A2, cv::Scalar::all(255));
  cv::line(mask, B1, B2, cv::Scalar::all(255));
  std::vector<cv::Point> correct_end_pts;
  correct_end_pts.push_back(A1);
  correct_end_pts.push_back(A2);
  correct_end_pts.push_back(B1);
  correct_end_pts.push_back(B2);
  test_EndFinder(mask, correct_end_pts);
}

TEST(TestSuite, EndFinder_cross) {
  test_EndFinder_cross(0);
  test_EndFinder_cross(1);
  test_EndFinder_cross(2);
}

////////////////////////////////////////////////////////////////////////////////

void test_EndFinder_line(int cols, int rows, cv::Point A, cv::Point B,
                         bool print_time = true) {
  cv::Mat1b mask(rows, cols, (uchar) 0);
  cv::line(mask, A, B, cv::Scalar::all(255));
  std::vector<cv::Point> correct_end_pts;
  correct_end_pts.push_back(A);
  correct_end_pts.push_back(B);
  test_EndFinder(mask, correct_end_pts, print_time);
}

TEST(TestSuite, EndFinder_line_small) {
  for (unsigned int i = 0; i < 20; ++i) {
    int cols = 10, rows = 10;
    cv::Point A(rand() % cols, rand() % rows), B = A;
    while (B == A)
      B = cv::Point(rand() % cols, rand() % rows);
    test_EndFinder_line(cols, rows, A, B, i == 0);
  } // end loop i
}

TEST(TestSuite, EndFinder_line_big) {
  for (unsigned int i = 0; i < 20; ++i) {
    int cols = 100, rows = 200;
    cv::Point A(rand() % cols, rand() % rows), B = A;
    while (B == A)
      B = cv::Point(rand() % cols, rand() % rows);
    test_EndFinder_line(cols, rows, A, B, i == 0);
  } // end loop i
}

TEST(TestSuite, EndFinder_skeleton) {
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "skeletons/heads/skeleton_sample.png", GRAYSCALE);
  // test_EndFinder(mask, end_pts);
  std::vector<cv::Point> end_pts;
  vision_utils::Timer timer;
  bool ok = vision_utils::detect_end_points(mask, end_pts);
  timer.printTime("vision_utils::detect_end_points()");
  ASSERT_TRUE(ok);
  ASSERT_TRUE(end_pts.size() == 10)
      << "end_pts:" << vision_utils::iterable_to_string(end_pts);

  //  cv::Mat3b viz;
  //  cv::cvtColor(mask, viz, CV_GRAY2BGR);
  //  for (unsigned int end_pt_idx = 0; end_pt_idx < end_pts.size(); ++end_pt_idx)
  //    cv::circle(viz, end_pts[end_pt_idx], 3, CV_RGB(0, 0, 255), 2);
  //  cv::imshow("viz", viz); cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, HighestPointFinder_arnaud1) {
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_canny.png", GRAYSCALE);
  cv::Point seed(433, 168), highest_pt;
  vision_utils::HighestPointFinder<uchar> finder;
  highest_pt = finder.find(mask, seed);
  cv::circle(mask, seed, 3, cv::Scalar::all(0), 2);
  cv::circle(mask, highest_pt, 3, cv::Scalar::all(120), 2);
if (display) {
  cv::imshow("mask", mask); cv::waitKey(0);
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, HighestPointFinder_arnaud3) {
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud3_canny.png", GRAYSCALE);
  cv::Point seed(290, 188), highest_pt;
  vision_utils::HighestPointFinder<uchar> finder;
  highest_pt = finder.find(mask, seed);
  cv::circle(mask, seed, 3, cv::Scalar::all(0), 2);
  cv::circle(mask, highest_pt, 3, cv::Scalar::all(120), 2);
if (display) {
  cv::imshow("mask", mask); cv::waitKey(0);
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, floodfill_edge_closer_david_arnaud1) {
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_canny.png", GRAYSCALE);
  vision_utils::FloodFillEdgeCloser closer;
  closer.close(mask, cv::Point(433, 168));
if (display) {
  cv::imshow("mask", mask); cv::waitKey(0);
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, floodfill_edge_closer_david_arnaud3) {
  cv::Mat1b mask = cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud3_canny.png", GRAYSCALE);
  cv::Point seed(290, 188);
  vision_utils::FloodFillEdgeCloser closer;
  closer.close(mask, seed, true, false, (uchar) 0, 50, .8f);
  cv::circle(mask, seed, 3, cv::Scalar::all(0), 2);
if (display) {
  cv::imshow("mask", mask); cv::waitKey(0);
    } // end if display
}
#endif // end of the skipped tests

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
