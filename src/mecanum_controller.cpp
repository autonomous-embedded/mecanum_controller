#include "mecanum_controller.hpp"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

#include <utility>

MecanumController::MecanumController(ros::NodeHandle nodeHandle) 
  : node(std::move(nodeHandle)),
    ultraFrontSub(node.subscribe("/front_sensor", 5, &MecanumController::UltraFrontCb, this)),
    ultraLeftSub(node.subscribe("/left_sensor", 5, &MecanumController::UltraFrontCb, this)),
    ultraRightSub(node.subscribe("/right_sensor", 5, &MecanumController::UltraFrontCb, this)),
    ultraBackSub(node.subscribe("/rear_sensor", 5, &MecanumController::UltraFrontCb, this)),
    odomSub(node.subscribe("/odom", 5, &MecanumController::OdomCb, this)),
    cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
    cmdVelPubRate(10),
    frontDistanceMeters{5.0},
    leftDistanceMeters{5.0},
    rightDistanceMeters{5.0},
    rearDistanceMeters{5.0},
    posX{0.0},
    posY{0.0} {}

namespace {
double CalculateLinearControlInRange(const double diff, const double minSpeed, 
                                     const double maxSpeed) {
  double speed = 0.5 * diff;

  if (diff > maxSpeed) {
    speed = maxSpeed;
  }
  else if (diff < minSpeed) {
    speed = minSpeed;
  }
  return speed;
}
} // namespace

void MecanumController::Run() {
  // TODO: do something more creative
  geometry_msgs::Twist msg;
  
  const auto xDiff = targetX - posX;
  const auto yDiff = targetY - posY;
  
  if (xDiff < 0.1 && yDiff < 0.1) {
    targetAchieved = true;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
  }
  else {
    msg.linear.x = CalculateLinearControlInRange(xDiff, -1.0, 1.0);
    msg.linear.y = CalculateLinearControlInRange(yDiff, -1.0, 1.0);
  }

  cmdVelPub.publish(msg);

  ros::spinOnce();
  cmdVelPubRate.sleep();
}

void MecanumController::SetTarget(const double x, const double y) {
  targetX = x;
  targetY = y;
  targetAchieved = false;
}

void MecanumController::UltraFrontCb(const sensor_msgs::Range::ConstPtr& msg) {
  frontDistanceMeters = msg->range;
}

void MecanumController::UltraLeftCb(const sensor_msgs::Range::ConstPtr& msg) {
  leftDistanceMeters = msg->range;
}

void MecanumController::UltraRightCb(const sensor_msgs::Range::ConstPtr& msg) {
  rightDistanceMeters = msg->range;
}

void MecanumController::UltraRearCb(const sensor_msgs::Range::ConstPtr& msg) {
  rearDistanceMeters = msg->range;
}

void MecanumController::OdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
}
