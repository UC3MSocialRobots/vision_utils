/*!
  \file        test_pie_chart_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/13

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

\todo Description of the file

 */

#include "combinatorics/combinatorics_utils.h"
#include "time/timer.h"
#include "pie_chart_utils.h"

template<class _T>
void test_pie1(const std::vector<_T> & values,
               const std::vector<std::string> & labels) {
  cv::Mat3b pie_img, pie_caption, pie_chart;

  unsigned int ntimes = 100;
  Timer timer;
  for (unsigned int time_idx = 0; time_idx < ntimes; ++time_idx)
    pie_chart_utils::make_pie(values, pie_img);
  timer.printTime_factor("make_pie()", ntimes);

  timer.reset();
  for (unsigned int time_idx = 0; time_idx < ntimes; ++time_idx)
    pie_chart_utils::make_caption(labels, pie_caption);
  timer.printTime_factor("make_caption()", ntimes);

  timer.reset();
  for (unsigned int time_idx = 0; time_idx < ntimes; ++time_idx)
    pie_chart_utils::make_pie_and_caption
        (values, labels, pie_chart, 500, 500, 1, 1, colormaps::index2grey_washed);
  timer.printTime_factor("make_pie_and_caption()", ntimes);

  // display generated images
  cv::imshow("pie_img", pie_img);
  cv::imshow("pie_caption", pie_caption);
  cv::imshow("pie_chart", pie_chart);
  cv::waitKey(0);
} // end if test_pie1()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void test_pie2(std::vector<_T> & values,
               const std::vector<std::string> & labels) {
  srand(time(NULL));
  cv::Mat3b pie_img, pie_caption, pie_chart;

  while (true) {
    // randomly change values
    for (unsigned int value_idx = 0; value_idx < values.size(); ++value_idx)
      values[value_idx] =
          std::max(values[value_idx] + combinatorics_utils::rand_gaussian() / 10, 0.);

    // draw and display generated images
    pie_chart_utils::make_pie(values, pie_img);
    pie_chart_utils::make_caption(labels, pie_caption);
    pie_chart_utils::make_pie_and_caption(values, labels, pie_chart);
    cv::imshow("pie_img", pie_img);
    cv::imshow("pie_caption", pie_caption);
    cv::imshow("pie_chart", pie_chart);
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
} // end if test_pie2()

////////////////////////////////////////////////////////////////////////////////

int main() {
  // make vectors from C arrays
  int nvals = 4;
  double values_arr[] = {.1, .2, .2, .4};
  std::string labels_arr[] = {"foo", "bar", "zim", "zam"};
  std::vector<double> values(values_arr, values_arr + nvals);
  std::vector<std::string> labels(labels_arr, labels_arr + nvals);

  test_pie1(values, labels);
  test_pie2(values, labels);
  return 0;
}

