#include <ros/console.h>
#include <ros/ros.h>

#include "mecanum_controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mecanum_controller");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh;
  MecanumController controller{nh};

  while (ros::ok()) {
    controller.Run();
  }

  return EXIT_SUCCESS;
}
