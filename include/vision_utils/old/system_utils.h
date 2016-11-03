#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

// C++
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <limits>       // std::numeric_limits
// C
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
// boost
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
// AD

namespace vision_utils {
//cut:safe_number_read_from_cin
/*!
  Read number values from cin.
  It is templated so it can read ints, doubles, etc.
  \return true if success
  */
template<class _T>
inline bool safe_number_read_from_cin(_T & dst) {
  // read line
  std::string line;
  std::getline(std::cin, line);
  // cast it to _T
  std::istringstream myStream(line);
  bool success = (myStream >> dst);
  return success;
} // end safe_number_read_from_cin()

////////////////////////////////////////////////////////////////////////////////
//cut:wait_for_key
//! wait for a key press and return
inline void wait_for_key () {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
  std::cout << std::endl << "Press any key to continue..." << std::endl;

  FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
  _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
  std::cout << std::endl << "Press ENTER to continue..." << std::flush;

  //  from http://www.cplusplus.com/forum/articles/7312/
  std::cin.clear();
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
#endif
}

////////////////////////////////////////////////////////////////////////////////
//cut:directory_exists
/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if the directory with given directoryname exists
*/
inline bool directory_exists(const std::string & directoryname) {
  return boost::filesystem::exists(directoryname)
      && boost::filesystem::is_directory(directoryname);
}
//cut:create_directory
/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if a new directory was created, otherwise false
*/
inline bool create_directory(const std::string & directoryname) {
  return boost::filesystem::create_directory(directoryname);
}
//cut:remove_directory
/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return The number of files removed (including the directory itself),
          i.e. 0 if the folder does not exist
*/
inline int remove_directory(const std::string & directoryname) {
  return boost::filesystem::remove_all(directoryname);
}

////////////////////////////////////////////////////////////////////////////////
//cut:remove_file
/*!
 \param filename
    a relative or absolute filename
 \return  false if filename did not exist in the first place, otherwise true.
*/
inline bool remove_file(const std::string & filename) {
  return boost::filesystem::remove(filename);
}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//cut:all_files_in_dir
/*!
 * Return the filename of all files in a given folder
 * \param folder
 *    the relative or absolute folder
 * \param ans
 *    the vector that will contain all answers
 * \param pattern
 *    the pattern of files to keep, for instance ".kpg"
 * \param add_folder_to_filename
 *    if true, \a folder will be added to each filename, f.ex. "/tmp/foo.dat",
 *    otherwise, just the raw filename is kept, f.ex. "foo.dat"
 * \return
 *    the number of files found.
 */
inline int all_files_in_dir(const std::string & folder,
                            std::vector<std::string> & ans,
                            const std::string pattern = "",
                            bool add_folder_to_filename = true) {
  ans.clear();
  std::string folder_and_slash = folder + std::string("/");

#if 1
  DIR *dir = opendir (folder.c_str());
  struct dirent *ent;
  if (dir == NULL) {
    printf("all_files_in_dir:could not open directory:'%s'\n", folder.c_str());
    return -1;
  }

  /* print all the files and directories within directory */
  while ((ent = readdir (dir)) != NULL) {
    //printf ("%s\n", ent->d_name);
    std::string filename(ent->d_name);
    if (pattern.size() > 0 && filename.find(pattern) == std::string::npos)
      continue;
    if (add_folder_to_filename)
      ans.push_back(folder_and_slash + filename);
    else
      ans.push_back(filename);
  }
  closedir (dir);
#elif 0
  std::ostringstream order;
  order << "ls -1 " << folder_and_slash << "*" << pattern << "*";
  std::string ls_res = exec_system_get_output(order.str().c_str());
  // string split
  ans.clear();
  if (ls_res == "")
    return 0;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= ls_res.size() - 1) {
    delim_pos = ls_res.find("\n", search_pos);
    //printf("delim_pos:%i, search_pos:%i", delim_pos, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      ans.push_back(ls_res.substr(search_pos));
      break;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      ans.push_back(ls_res.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + 1; // size of delim
    // quit if we reached the end of the std::string or std::string empty
  }
#else // https://stackoverflow.com/questions/4283546/cboostfilesystem-how-can-i-get-a-list-of-files-in-a-folder-in-which-the-fi
  boost::filesystem::path someDir(folder);
  boost::filesystem::directory_iterator end_iter;

  typedef std::multimap<std::time_t, boost::filesystem::path> result_set_t;
  result_set_t result_set;

  if ( boost::filesystem::exists(someDir) && boost::filesystem::is_directory(someDir)) {
    for( boost::filesystem::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter) {
      if (boost::filesystem::is_regular_file(dir_iter->status()) ) {
        result_set.insert(result_set_t::value_type(boost::filesystem::last_write_time(dir_iter->status()), *dir_iter);
      }
    }
  }
#endif
  return ans.size();
} // end all_files_in_dir()

//cut
} // end namespace vision_utils

#endif // SYSTEM_UTILS_H
