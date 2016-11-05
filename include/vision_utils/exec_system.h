/*!
  \file        exec_system.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/29
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

#ifndef EXEC_SYSTEM_H
#define EXEC_SYSTEM_H

#include <stdio.h>

namespace vision_utils {

//! execute a system instruction in a safe mode
inline int exec_system(const std::string & instr) {
  int return_value = system(instr.c_str());
  if (return_value < 0) {
    printf("system('%s') returned %i < 0!\n", instr.c_str(), return_value);
  }
  return return_value;
} // end system()

} // end namespace vision_utils

#endif // EXEC_SYSTEM_H
