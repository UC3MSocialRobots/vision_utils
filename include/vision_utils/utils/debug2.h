/*!
  \file        debug2.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/8
  
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

Some useful functions for debugging a part of the code.
Put "#define debug" befure this include to make them visible in the ouput.
        
 */

#ifndef debug2_H
#define debug2_H

#ifdef debug

#include <string>
#include <stdio.h> // for fprintf

/*! Stringified __LINE__ */
#define debug_STRINGIFY(x) #x
#define debug_TO_STRING(x) debug_STRINGIFY(x)
//Colors
#define PRINTF_FORMAT_NO_ATTRIB 0
#define PRINTF_FORMAT_BOLD 1
#define PRINTF_FORMAT_UNDERLINE 4
#define PRINTF_FORMAT_BLACK 30
#define PRINTF_FORMAT_RED 31
#define PRINTF_FORMAT_GREEN 32
#define PRINTF_FORMAT_YELLOW 33
#define PRINTF_FORMAT_BLUE 34
#define PRINTF_FORMAT_MAGENTA 35
#define PRINTF_FORMAT_CYAN 36
#define PRINTF_FORMAT_WHITE 37

#define debug_FILE_POSITION \
  (  ((std::string) __FILE__).substr(1 + ((std::string) __FILE__).find_last_of('/')) \
  + ":" + debug_TO_STRING(__LINE__) )

#ifdef __GNUC__
#define __debug_HERE__ ( std::string ("[") + debug_FILE_POSITION + std::string (" (") + std::string(__FUNCTION__) + std::string(")] \t") )
#else
#define __debug_HERE__ ( std::string ("[") + debug_FILE_POSITION + std::string ("] \t") )
#endif

#define debug_PRINTF(printf_attr, printf_color, ...) { \
  printf("%c[%d;%dm%s", 0x1B, printf_attr, printf_color, __debug_HERE__.c_str()); \
  printf("%c[%dm", 0x1B, PRINTF_FORMAT_NO_ATTRIB); \
  printf(__VA_ARGS__); \
  }

#define debugPrintf( ... ) { \
    debug_PRINTF(PRINTF_FORMAT_BOLD, PRINTF_FORMAT_RED, __VA_ARGS__) \
  }
    //printf("\n");

#define debugStreamNoHeader( ... ) { \
  std::cout << __VA_ARGS__; \
  }

#define debugStream( ... ) { \
  debug_PRINTF(PRINTF_FORMAT_BOLD, PRINTF_FORMAT_RED, " ") \
  debugStreamNoHeader(__VA_ARGS__ << std::endl); \
  }

#else // no debug

#define debugPrintf( ... ) {}
#define debugStream( ... ) {}
#define debugStreamNoHeader( ... ) {}

#endif

#endif // debug2_H
