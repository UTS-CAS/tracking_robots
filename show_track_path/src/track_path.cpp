#include "show_track_path/track_path.h"

track_path::track_path(ros::NodeHandle& nh):_mm_per_pixel(0.5),_location(" "),_endpoint_name(" "),_pub_hz(10),_first(false)
{
  _pnh = new ros::NodeHandle(nh);

  if(!checkParam())
  {
    ROS_ERROR("Check launch parameters.");
    return;
  }

  //load img here into mat
  _track_mat = cv::imread(_location.c_str());

  //choose start point interactively(?)
//  _start_col = 100;
//  _start_row = 70;

  //sub so that relates current joint pose to start point in matfile
  _endpoint_sub = _pnh->subscribe(_endpoint_name.c_str(),1,&track_path::firstCB,this);
  while(ros::ok() && !_first)
  {
    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }

  if(!_first)
  {
    return;
  }

  ROS_INFO("Start x: %f, start y: %f",_startx,_starty);
  ROS_INFO("Waiting for 1 second.");
  ros::Duration(1.0).sleep();
  _loop = new ros::Rate(_pub_hz);
  _endpoint_sub = _pnh->subscribe(_endpoint_name.c_str(),1,&track_path::endpointCB,this);

  cv::namedWindow("track",CV_WINDOW_NORMAL);
  cv::imshow("track",_track_mat);
  cv::waitKey(100);

  while(ros::ok() && _pnh->ok())
  {
    ros::spinOnce();
    _loop->sleep();
  }
}

bool track_path::checkParam()
{
  bool temp = true;
  if(!_pnh->getParam("mm_per_pixel",_mm_per_pixel))
  {
    ROS_INFO("No mm per pixel parameter found.");
    temp = false;
  }
  if(!_pnh->getParam("img_location",_location))
  {
    ROS_INFO("No img location parameter found.");
    temp = false;
  }
  if(!_pnh->getParam("endpoint_topic_name",_endpoint_name))
  {
    ROS_INFO("No endpoint topic name parameter found.");
    temp = false;
  }
  if(!_pnh->getParam("col_multiplier",_col_mult))
  {
    ROS_INFO("No y multiplier parameter found.");
    temp = false;
  }
  if(!_pnh->getParam("row_multiplier",_row_mult))
  {
    ROS_INFO("No x multiplier parameter found.");
    temp =false;
  }
  if(!_pnh->getParam("start_row",_start_row))
  {
    ROS_INFO("No start_row parameter found.");
    temp = false;
  }
  if(!_pnh->getParam("start_col",_start_col))
  {
    ROS_INFO("No start_col parameter found.");
    temp = false;
  }
  if(_pnh->getParam("publish_rate",_pub_hz))
  {
    ROS_INFO("Default rate of %dHz used.",_pub_hz);
  }
  return temp;
}

void track_path::updateMat(double xval,double yval)
{
  cv::Mat temp;
  _track_mat.copyTo(temp);
  cv::Point cent;

  double deltax = (xval-_startx)*1000;
  double deltay = (yval-_starty)*1000;
  int pixelrow = deltax/_mm_per_pixel;
  int pixelcol = deltay/_mm_per_pixel;

  cent.x = _start_col+(_col_mult)*pixelcol;
  cent.y = _start_row+(_row_mult)*pixelrow;

  cv::circle(temp,cent,15,cv::Scalar(0,0,255),5);
  cv::circle(temp,cent, 5,cv::Scalar(0,0,255),2);
  cv::imshow("track",temp);
  cv::waitKey(100);
}

void track_path::endpointCB(const intera_core_msgs::EndpointStateConstPtr &msg)
{
  updateMat(msg->pose.position.x,msg->pose.position.y);
}

void track_path::firstCB(const intera_core_msgs::EndpointStateConstPtr &msg)
{
  if(!_first)
  {
    _startx = msg->pose.position.x;
    _starty = msg->pose.position.y;
    _first = true;
  }
}
