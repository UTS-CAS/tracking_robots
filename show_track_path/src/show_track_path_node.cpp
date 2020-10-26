#include <show_track_path/track_path.h>



int main(int argc,char** argv)
{
  ros::init(argc,argv,"show_track_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  track_path test(pnh);

  return 0;
}
