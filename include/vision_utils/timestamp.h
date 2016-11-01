/*!
  \file        timestamp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/6

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

Some useful functions for strings bound with time.
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <sys/time.h>
#include <string>
#include <sstream>
#include <iomanip>      // std::setfill, std::setw

namespace vision_utils {

/*!
 \return std::string
    the current time, in format: yy-mm-dd_hh-mn-sc-msec
    It only contains characters friendly with a filename.
 \see http://www.cplusplus.com/reference/clibrary/ctime/strftime/
*/
inline std::string timestamp() {
#if 0 // strftime, does not support microseconds
  time_t rawtime;
  time ( &rawtime );
  struct tm * timeinfo = localtime ( &rawtime );
  char buffer [80];
  strftime (buffer,80,"%Y-%m-%d_%H-%M-%S", timeinfo);
  //puts (buffer);
  return std::string(buffer);
#else
  /* Obtain the time of day, and convert it to a tm struct. */
  struct timeval tv;
  gettimeofday (&tv, NULL);
  struct tm* ptm = localtime (&tv.tv_sec);
  /* Format the date and time, down to a single second. */
  char time_string[40];
  strftime (time_string, sizeof (time_string), "%Y-%m-%d_%H-%M-%S", ptm);
  /* Compute milliseconds from microseconds. */
  int milliseconds = (int) tv.tv_usec / 1000;
  std::ostringstream ans;
  // add the milliseconds with eventual leading zeros
  ans << time_string << "-"
      << std::setw( 3 ) << std::setfill( '0' ) << milliseconds;
  return ans.str();
#endif
} // end timestamp()

} // end namespace vision_utils

#endif // TIMESTAMP_H
