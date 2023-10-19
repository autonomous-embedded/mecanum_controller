#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

class MecanumController {
  // node handle
  ros::NodeHandle node;

  // sub
  ros::Subscriber ultraFrontSub;
  ros::Subscriber ultraLeftSub;
  ros::Subscriber ultraRightSub;
  ros::Subscriber ultraBackSub;
  ros::Subscriber depthSub;
  ros::Subscriber colorSub;

  // pub, timing
  // cmd_vel
  ros::Publisher cmdVelPub;
  ros::Rate cmdVelPubRate;

  // data & stuff -> updated a lot
  double frontDistanceMeters;
  double leftDistanceMeters;
  double rightDistanceMeters;
  double rearDistanceMeters;

 public:
  MecanumController(ros::NodeHandle nodeHandle);

  void Run();

 private:
  void UltraFrontCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraLeftCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraRightCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraRearCb(const sensor_msgs::Range::ConstPtr& msg);
};

#endif // MECANUM_CONTROLLER_HPP_