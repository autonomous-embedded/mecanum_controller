#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <vector>

class MecanumController {
  // node handle
  ros::NodeHandle node;

  // sub
  ros::Subscriber ultraFrontSub;
  ros::Subscriber ultraLeftSub;
  ros::Subscriber ultraRightSub;
  ros::Subscriber ultraBackSub;
  ros::Subscriber odomSub;
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
  double posX;
  double posY;

  // setpoint (odometry ctrl)
  double targetX;
  double targetY;
  bool targetAchieved;

 public:
  MecanumController(ros::NodeHandle nodeHandle);

  void Run();

  void SetTarget(const double x, const double y);
  bool GetTargetAchieved() const { return targetAchieved; }

 private:
  void UltraFrontCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraLeftCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraRightCb(const sensor_msgs::Range::ConstPtr& msg);
  void UltraRearCb(const sensor_msgs::Range::ConstPtr& msg);
  void OdomCb(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif // MECANUM_CONTROLLER_HPP_