#include <track_robot/track_robot.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"track_robot_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  track_robot test(pnh);

  test.run();
  return 0;
}
