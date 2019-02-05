#ifndef TRACK_ROBOT_H
#define TRACK_ROBOT_H
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <ros/subscriber.h>
#include <mutex>
#include <algorithm>
#include <utility>

#include <track_robot_msgs/Trigger.h>
#include <intera_core_msgs/EndpointState.h>


class track_robot
{
public:
  track_robot(ros::NodeHandle pnh);

  void run();

private:
  struct subs{
    ros::Subscriber _sub_endpoint;
    ros::Subscriber _sub_trigger;
    std::mutex _mtx;
    int _buffer;
  };

  ros::NodeHandle* _pnh;
  ros::Rate* _loop;

//  rosbag::Bag* _bag;
  std::pair<rosbag::Bag,std::mutex>* _bags;
  subs* _subs;
  std::string _endpoint_name;
  std::string _trigger_name;
  std::string _rosbag_name;
  int _sub_buffer;
  bool _tested,_finished;

  bool loadParams();
  void endpointCB(const intera_core_msgs::EndpointStateConstPtr &endpoint);
  void triggerCB(const track_robot_msgs::TriggerConstPtr &trig);
};

#endif // TRACK_ROBOT_H
