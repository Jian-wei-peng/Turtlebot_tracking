#include "me5413/path_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_publisher_node");
  me5413::PathPublisher path_publisher_node;
  ros::spin();
  return 0;
}