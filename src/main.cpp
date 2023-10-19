#include "mecanum_controller.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mecanum_controller");
  
  ros::NodeHandle nh;
  MecanumController controller{nh};
  
  const double targetX1 = 10.0;
  const double targetY1 = 10.0;
  const double targetX2 = 0.0;
  const double targetY2 = 0.0;

  controller.SetTarget(targetX1, targetY1);

  while (ros::ok()) {
    if (controller.GetTargetAchieved()) {
      controller.SetTarget(targetX2, targetY2);
    }
    controller.Run();
  }

  return EXIT_SUCCESS;
}