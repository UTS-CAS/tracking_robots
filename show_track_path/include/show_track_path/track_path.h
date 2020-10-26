#ifndef TRACK_PATH_H
#define TRACK_PATH_H
#include <ros/ros.h>

#include <intera_core_msgs/EndpointState.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class track_path
{
public:
  track_path(ros::NodeHandle& nh);

private:
  ros::NodeHandle* _pnh;
  ros::Subscriber _endpoint_sub;
  ros::Rate* _loop;


  int _pub_hz,_start_row,_start_col,_col_mult,_row_mult;
  bool _first;
  double _mm_per_pixel,_startx,_starty;
  std::string _location,_endpoint_name;
  cv::Mat _track_mat;

  bool checkParam();
  void updateMat(double xval,double yval);
  void endpointCB(const intera_core_msgs::EndpointStateConstPtr &msg);
  void firstCB(const intera_core_msgs::EndpointStateConstPtr &msg);
};

#endif // TRACK_PATH_H
