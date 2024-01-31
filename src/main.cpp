#include <ros/console.h>
#include <ros/ros.h>

#include "mecanum_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mecanum_controller");

  ros::console::notifyLoggerLevelsChanged();

  ros::NodeHandle nodeHandle;
  MecanumController controller{nodeHandle};

  while (ros::ok()) {
    controller.Run();
  }

  return EXIT_SUCCESS;
}
