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
    cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
    cmdVelPubRate(10) {
  frontDistanceMeters = 5.0;
  leftDistanceMeters = 5.0;
  rightDistanceMeters = 5.0;
  rearDistanceMeters = 5.0;
}

void MecanumController::Run() {
  // TODO: do something more creative
  geometry_msgs::Twist msg;
  msg.linear.x = 1.0;

  if (frontDistanceMeters < 1.0) {
    msg.linear.x = 0;
  }

  cmdVelPub.publish(msg);

  ros::spinOnce();
  cmdVelPubRate.sleep();
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
