/*!
  \file        is_process_alive.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/30
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

#ifndef IS_PROCESS_ALIVE_H
#define IS_PROCESS_ALIVE_H

#include <vision_utils/exec_system_get_output.h>

namespace vision_utils {

/*!
 \param process_pid
    the PID of the process
 \return trueif the process with given PID exists and is alive
*/
inline bool is_process_alive(const int & process_pid) {
  //printf("is_process_alive(%i)", process_pid);
  std::ostringstream process_pid_str;
  process_pid_str << process_pid;
  // exec ps -p 1234
  std::ostringstream instr;
  instr << "ps -p " << process_pid_str.str();
  std::string output = exec_system_get_output(instr.str().c_str());
  /* if the process exists, get like
  $ ps -P 4839
      PID PSR TTY      STAT   TIME COMMAND
     4839   1 ?        Sl     0:01 geany
  Otherwise:
      PID PSR TTY      STAT   TIME COMMAND
   */
  return output.find(process_pid_str.str()) != std::string::npos;
} // end is_process_alive()

} // end namespace vision_utils

#endif // IS_PROCESS_ALIVE_H
