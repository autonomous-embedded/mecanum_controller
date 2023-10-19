#include "mecanum_controller.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mecanum_controller");
  
  ros::NodeHandle nh;
  MecanumController controller{nh};
  
  while (ros::ok()) {
    controller.Run();
  }

  return EXIT_SUCCESS;
}