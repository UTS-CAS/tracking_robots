#include "track_robot/track_robot.h"

track_robot::track_robot(ros::NodeHandle pnh):_sub_buffer(0),_tested(false),_finished(false)
{
  _pnh = new ros::NodeHandle(pnh);

  if(!loadParams())
  {
    ROS_ERROR("Error in loading parameters");
    return;
  }

  _loop = new ros::Rate(150);
  _subs = new subs;
  _bags = new std::pair<rosbag::Bag,std::mutex>;

  _subs->_sub_trigger = _pnh->subscribe<track_robot_msgs::Trigger>(_trigger_name.c_str(),1,&track_robot::triggerCB,this);

  _bags->first.open(_rosbag_name.c_str(),rosbag::bagmode::Write);
}

bool track_robot::loadParams()
{
  bool temp = true;
  if(!_pnh->getParam("endpoint_topic_name",_endpoint_name))
  {
    ROS_ERROR("No endpoint topic name given.");
    temp = false;
  }
  if(!_pnh->getParam("trigger_topic_name",_trigger_name))
  {
    ROS_ERROR("No trigger topic name given.");
    temp = false;
  }
  if(!_pnh->getParam("rosbag_name",_rosbag_name))
  {
    ROS_ERROR("No rosbag name given.");
    temp = false;
  }
  if(!_pnh->getParam("trigger_buffer",_sub_buffer))
  {
    ROS_ERROR("No trigger buffer value given.");
    temp = false;
  }
  return temp;
}


void track_robot::endpointCB(const intera_core_msgs::EndpointStateConstPtr &endpoint)
{
  std::unique_lock<std::mutex> lock(_bags->second);
  _bags->first.write("EndpointState",ros::Time::now(),*endpoint);
  lock.unlock();
}

void track_robot::triggerCB(const track_robot_msgs::TriggerConstPtr &trig)
{
  if(trig->trig && !_tested)
  {
    std::unique_lock<std::mutex> lock(_subs->_mtx);
    _subs->_sub_endpoint = _pnh->subscribe(_endpoint_name.c_str(),1,&track_robot::endpointCB,this);
    _subs->_buffer = 0;
    lock.unlock();
    _tested = true;
    ROS_INFO("Started subscribing to sawyer arm");
  }
  else if(!trig->trig && _tested)
  {
    _subs->_buffer++;
    if(_subs->_buffer > _sub_buffer)
    {
      std::unique_lock<std::mutex> lock(_subs->_mtx);
      _subs->_sub_endpoint.shutdown();
      lock.unlock();
      ROS_INFO("Stopped subscription to sawyer arm");
      _finished = true;
    }
  }
}


void track_robot::run()
{
  ROS_INFO("Running after a 1 second delay!");
  ros::Duration(1.0).sleep();
  while(ros::ok() && _pnh->ok())
  {
    ros::spinOnce();

    if(_finished)
    {
      ROS_INFO("NodeHandle detected to be off. Closing rosbag file.");
      _bags->first.close();
      ROS_INFO("Finished closing rosbag file.");
      ros::shutdown();
      ros::waitForShutdown();
    }
    else
    {
      _loop->sleep();
    }
  }
}
