/*!
  \file        test_timer_chart.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/14

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

#define CHART_TIMER_ON
#include "timer_chart.h"
// put the line "#define CHART_TIMER_ON" in your code to activate the TimerChart

int main() {
  TIMER_CREATE(timer);
  while (true) {
    usleep(1000 * (combinatorics_utils::rand_gaussian() * 20 + 100));
    TIMER_PRINT_RESET(timer, "foo");
    usleep(1000 * (combinatorics_utils::rand_gaussian() * 20 + 200));
    TIMER_PRINT_RESET(timer, "bar");
    usleep(1000 * (combinatorics_utils::rand_gaussian() * 20 + 100));
    TIMER_PRINT_RESET(timer, "zim");
    usleep(1000 * (combinatorics_utils::rand_gaussian() * 20 + 500));
    TIMER_PRINT_RESET(timer, "zam");
    TIMER_DISPLAY_CHART(timer, 1);
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
  return 0;
}

