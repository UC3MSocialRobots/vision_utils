/*!
  \file        timer_chart.h
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

#ifndef TIMER_CHART_H
#define TIMER_CHART_H

#include <map>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/timer.h"
#include "vision_utils/pie_chart_utils.h"

// some useful preprocesor functions for TimerCharts.
// put the line "#define CHART_TIMER_ON" in your code,
// BEFORE the include of "timer_chart.h", to activate the TimerChart
#ifdef CHART_TIMER_ON
#define TIMER_CREATE(t)                     ::vision_utils::TimerChart t;
#define TIMER_RESET(t)                      t.reset();
#define TIMER_PRINT_RESET(t, label)           t.printTime(label); t.reset();
#define TIMER_DISPLAY_CHART(t, step)        t.display_chart(step);
#else // do nothing
#define TIMER_CREATE(t)                     // empty
#define TIMER_RESET(t)                      // empty
#define TIMER_PRINT_RESET(t, label)           // empty
#define TIMER_DISPLAY_CHART(t, step)        // empty
#endif // CHART_TIMER_ON

namespace vision_utils {

class TimerChart : public Timer {
public:
  typedef std::map<std::string, int> Label2IndexMap;

  TimerChart(const std::string & window_name = "TimerChart") :
    Timer(), _window_name(window_name) {
    _display_call_counter = 0;
    cv::namedWindow(_window_name);
  }

  //! print time needed for a task identified by its string
  virtual inline void printTime(const char* label) {
    // unordered labels
    // store time for this phase
    //_times[std::string(label)] = getTimeMilliseconds();

    // ordered labels
    // did we already have this label?
    Label2IndexMap::const_iterator it = _label2indices.find(std::string(label));
    if (it == _label2indices.end()) { // store the new label
      int new_idx = _times.size();
      _times.push_back(getTimeMilliseconds());
      _label2indices[std::string(label)] = new_idx;
      _labels_vector_dump.push_back(std::string(label));
    }
    else { // the label already existed
      _times[it->second] = getTimeMilliseconds();
    }
      //printf("labelled_times:'%s'\n", map_to_string(_times).c_str());
  }

  //! display chart, but not for each call
  virtual inline void display_chart(const int display_step = 1) {
    if (_display_call_counter++ % display_step == 0) {
      //      _labels_vector_dump.resize(_label2indices.size());
      //      for( Label2IndexMap::const_iterator it = m.begin();
      //           it != m.end(); ++it ) {
      //        _labels_vector_dump[it -> second] = it->first;
      //      }

      //map_keys_to_container(_times, _labels_vector_dump);
      make_pie_and_caption(_times, _labels_vector_dump,
                                            _chart, 800, 600, false, .08);
      cv::imshow(_window_name, _chart);
      cv::waitKey(5);
    }
  }

private:
  std::string _window_name;
  std::vector<Time> _times;
  Label2IndexMap _label2indices;
  std::vector<std::string> _labels_vector_dump;
  int _display_call_counter;
  cv::Mat3b _chart;
};

} // end namespace vision_utils

#endif // TIMER_CHART_H
