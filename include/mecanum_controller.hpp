#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
// #include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <vector>

class MecanumController {
  static constexpr int H_MIN_ORANGE{0}, S_MIN_ORANGE{100}, V_MIN_ORANGE{50};
  static constexpr int H_MAX_ORANGE{30}, S_MAX_ORANGE{255}, V_MAX_ORANGE{255};

  static constexpr int L_MIN_ORANGE{25}, A_MIN_ORANGE{150}, B_MIN_ORANGE{150};
  static constexpr int L_MAX_ORANGE{255}, A_MAX_ORANGE{230}, B_MAX_ORANGE{230};

  // node handle
  ros::NodeHandle node;

  // sub
  ros::Subscriber odomSub;
  ros::Subscriber depthSub;
  ros::Subscriber colorSub;

  // pub, timing
  // cmd_vel
  ros::Publisher detPub;
  ros::Publisher cmdVelPub;
  ros::Rate cmdVelPubRate;

  // data & stuff -> updated a lot
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
  void OdomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void ColorImgCb(const sensor_msgs::ImagePtr& msg);
};

#endif // MECANUM_CONTROLLER_HPP_