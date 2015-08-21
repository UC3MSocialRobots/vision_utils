/*!
  \file        std_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/21

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

Some useful functions shared for the project kinect

 */

#ifndef STD_UTILS_H
#define STD_UTILS_H

// std
#include <fstream>
// boost
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
// ROS
#include <std_msgs/ColorRGBA.h>
#include <ros/console.h>

namespace std_utils {

//! \return \true if distance is a NAN (or a simulated one)
inline bool is_nan_depth(const float & distance) {
  return (distance == 0 || isnan(distance));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// system utils

//! execute a system instruction in a safe mode
inline int exec_system(const std::string & instr) {
  int return_value = system(instr.c_str());
  if (return_value < 0) {
    ROS_WARN("system('%s') returned %i < 0!", instr.c_str(), return_value);
  }
  return return_value;
} // end system()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Execute a command and get its output.
 * Careful though, be aware that this will only grab stdout and not stderr
 * From http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
 * \param cmd
 * \return
 *    the output of that command
 */
inline std::string exec_system_get_output(const char* cmd) {
  ROS_DEBUG("exec_system_get_output('%s')", cmd);
  FILE* pipe = popen(cmd, "r");
  if (!pipe) {
    ROS_WARN("exec_system_get_output('%s'): could not open pipe", cmd);
    return "ERROR";
  }
  char buffer[128];
  std::string result = "";
  while(!feof(pipe)) {
    if(fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);
  return result;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param filename
    a relative or absolute filename
 \return true if the file with given filename exists
*/
inline bool file_exists(const std::string & filename) {
  return boost::filesystem::exists(filename)
      && boost::filesystem::is_regular_file(filename);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// map_utils
/*!
 * Perform a direct search on a map.
 * \param map
 *   The map to be searched.
 * \param search_key
 *   The key we want.
 * \param value_lookup_result (out)
 *   The value corresponding to \a search_key in the map,
 *   if \a search_key is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_key was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_key=0: returns false, value_lookup_result not affected
 *  search_key=1: returns true, value_lookup_result="value1"
 *  search_key=2: returns true, value_lookup_result="value2"
 */
template<class _Key, class _Value>
inline bool direct_search(const std::map<_Key, _Value> & map,
                           const _Key & search_key,
                           _Value & value_lookup_result) {
  typename std::map<_Key, _Value>::const_iterator it = map.find(search_key);
  if (it == map.end()) {
    return false;
  }
  value_lookup_result = it->second;
  return true;
} // end direct_search()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Perform an inverse search on a map.
 * \param map
 *   The map to be searched.
 * \param search_value
 *   The value we want.
 * \param key_lookup_result (out)
 *   The key corresponding to \a search_value in the map,
 *   if \a search_value is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_value was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_value="value0": returns false, key_lookup_result not affected
 *  search_value="value1": returns true, key_lookup_result=1
 *  search_value="value2": returns true, key_lookup_result=2
 */
template<class _Key, class _Value>
inline bool reverse_search (const std::map<_Key, _Value> & map,
                            const _Value & search_value,
                            _Key & key_lookup_result) {
  for (typename std::map<_Key, _Value>::const_iterator map_iterator = map.begin();
       map_iterator != map.end();
       map_iterator ++) {
    if (map_iterator->second == search_value) {
      key_lookup_result = map_iterator->first;
      return true;
    }
  } // end loop map_iterator
  return false;
} // end reverse_search()

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// string utils

/*! find all the iterations of a pattern in a string and replace
 * them with another pattern
 * \param stringToReplace the string to change
 * \param pattern what we want to replace
 * \param patternReplacement what we replace with
 * \return the number of times we replaced pattern
 */
inline int find_and_replace(std::string& stringToReplace,
                            const std::string & pattern,
                            const std::string & patternReplacement) {
  size_t j = 0;
  int nb_found_times = 0;
  for (; (j = stringToReplace.find(pattern, j)) != std::string::npos;) {
    //cout << "found " << pattern << endl;
    stringToReplace.replace(j, pattern.length(), patternReplacement);
    j += patternReplacement.length();
    ++ nb_found_times;
  }
  return nb_found_times;
}

////////////////////////////////////////////////////////////////////////////////

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
inline bool retrieve_file(const std::string & filepath, std::string & ans) {
  // open the file
  std::ifstream myfile(filepath.c_str(), std::ios::in);
  /*
   * check if success
   */
  if (!myfile || myfile.is_open() == false) {
    // error while reading the file
    ROS_WARN("Unable to open file '%s'", filepath.c_str());
    return false;
  }
  /*
   * concatenate to buffer
   */
  std::string line;
  std::ostringstream buffer;
  bool first_line = true;
  while (myfile.good()) {
    // add a cariage return if it is not the first line
    if (first_line)
      first_line = false;
    else
      buffer << std::endl;

    getline(myfile, line);
    buffer << line;
  } // end myfine.good()
  myfile.close();
  ans = buffer.str();
  return true;
} // end retrieve_file()

////////////////////////////////////////////////////////////////////////////////

/*! Save a string to a given file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline void save_file(const std::string & filepath, const std::string & content) {
  // maggieDebug2("save_file('%s')", filepath.c_str());
  std::ofstream myfile(filepath.c_str());
  // check if success
  if (myfile.is_open())
    myfile << content;
  else {// error while reading the file
    ROS_WARN("Unable to open file '%s' for writing.", filepath.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// string_utils_ros

template<class _T>
std::string ros_object_to_string(const _T & object) {
  uint32_t serial_size = ros::serialization::serializationLength(object);
  uint8_t buffer_uint[serial_size];
  ros::serialization::OStream stream_out(buffer_uint, serial_size);
  ros::serialization::Serializer<_T>::write(stream_out, object);
  return std::string((char*) buffer_uint, serial_size);
} // end ros_object_to_string()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void ros_object_to_file(const std::string & filename,
                        const _T & object) {
  save_file(filename, ros_object_to_string(object));
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void ros_object_from_string(const std::string & object_str,
                            _T & my_value) {
  uint32_t serial_size2 = object_str.size();
  ros::serialization::IStream stream_in((unsigned char*) object_str.data(), serial_size2);
  ros::serialization::Serializer<_T>::read(stream_in, my_value);
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
void ros_object_from_file(const std::string & filename,
                          _T & object) {
  std::string object_str;
  retrieve_file(filename, object_str);
  ros_object_from_string(object_str, object);
}
} // end namespace std_utils

#endif // STD_UTILS_H
