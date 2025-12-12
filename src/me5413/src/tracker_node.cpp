#include "me5413/path_tracker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413::PathTracker path_tracker_node;
  ros::spin();
  return 0;
}