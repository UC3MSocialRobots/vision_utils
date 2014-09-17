#ifndef NANO_ETTS_API_H
#define NANO_ETTS_API_H

#include <ros/node_handle.h>
#include <std_msgs/String.h>

/*!
 * \class NanoEttsApi
 * a tiny API for using etts that comes with no dependencies.
 * It communicates with the API using std_msgs/String messages.
 * Handy for saying a few sentences here and there
 * without depending on etts.
 */
class NanoEttsApi {
public:
  NanoEttsApi() {
  }

  void advertise() {
    _pub = _nh_public.advertise<std_msgs::String>("ETTS_SAY_PLAIN_TEXT", 100);
    sleep(1);
  }

  void shutdown() {
    _pub.shutdown();
  }

  inline void say_text(const std::string & sentence) {
    printf("NanoEttsApi:say_text('%s')\n", sentence.c_str());
    if (_pub.getTopic() == "")
      advertise();
    std_msgs::String msg;
    msg.data = sentence;
    _pub.publish(msg);
  }

  ros::NodeHandle _nh_public;
  ros::Publisher _pub;
}; // end class NanoEttsApi

#endif // NANO_ETTS_API_H
