/*!
 * Exceptions and macros for reporting errors.
 * \file error.h
 * \author Arnaud Ramey
 */

#ifndef _MAGGIE_CORE_ERROR_H_
#define _MAGGIE_CORE_ERROR_H_

// maggie
#include "vision_utils/utils/debug_utils.h"

// Stl & Std
#include <iostream>
#include <stdio.h>   // for sprintf()
#include <stdlib.h>  // for exit()
#include <stdarg.h>  // for vargs

/*!
 * The generic exception class.
 */
class Exception: public std::exception {
public:

  /*! Default constructor. */
  Exception() {
  }

  /*! Constructor setting values of the error location and message. */
  Exception(const std::string& where, const std::string& error) {
    message = where + formatMessage(error);
  }

  /*! Destructor. */
  virtual ~Exception() throw () {
  }

  /*! The final formatted message that should be displayed. */
  std::string message;

  /*! Returns the output error message. */
  virtual const char *what() const throw () {
    return message.c_str();
  }

  /*!
     * Creates formatted error message based on the information
     * in the exception.
     */
  virtual std::string formatMessage(std::string error) throw () {
    return "Error: " + error;
  }
};

/*!
 * Allows to format an error message in the printf-like fashion.
 * From OpenCV.
 */
static inline std::string errMsg(const char* fmt, ...) {
  char buf[1 << 16];
  va_list args;
  va_start( args, fmt );
  vsprintf(buf, fmt, args);
  return std::string(buf);
}

/*!
 * Routine used for fatal errors.
 * Displays messages and kills the process.
 * \param exc the Exception to cast
 */
static inline void exitWithError(const Exception& exc) {
  std::cout << exc.message << std::endl;
  exit(-1);
}


#ifdef AD_USE_ROS
#define maggieWeakError( ... )  ROS_ERROR(__VA_ARGS__)
#else  // AD_USE_ROS
#define maggieWeakError( ... )  MAGGIE_debug(PRINTF_FORMAT_UNDERLINE, PRINTF_FORMAT_RED, MARKER_PRINT, __VA_ARGS__)
#endif  // AD_USE_ROS

/*!
 * Macro reporting a fatal error.
 */
//#define maggieError( ... )
//        exitWithError(Exception(__MAGGIE_HERE__, errMsg(__VA_ARGS__)) )

#ifdef AD_USE_ROS
#define maggieError( ... ) { ROS_FATAL(__VA_ARGS__);  exit(-1); }
#else  // AD_USE_ROS
#define maggieError( ... ) { maggiePrint(__VA_ARGS__);  exit(-1); }
#endif  // AD_USE_ROS

#define maggieErrorNotImplemented() { \
  maggieError("Function not implemented.");\
  }

/*!
 * Macro defining an assertion.
 */
#define maggieAssert( expr ) { if(!(expr)) \
  exitWithError( Exception(__MAGGIE_HERE__, #expr) ); }

/*! Convenience macro throwing a general exception. */
#define maggieException( ... ) {\
  Exception e (__MAGGIE_HERE__, errMsg(__VA_ARGS__)); \
  maggiePrint("Exception '%s'", e.what()); \
  throw e; \
  }
/*!
 * Macro defining an assertion used only when debugging.
 */
#define maggieDebugAssert( expr ) { if ((MAGGIE_debug_LEVEL > 0 ) && (!(expr))) \
  exitWithError( Exception(__MAGGIE_HERE__, #expr) ); }

#endif /* MAGGIE_CORE_ERROR_H_ */

