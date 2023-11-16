#include "mecanum_controller.hpp"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <utility>
#include <vector>

MecanumController::MecanumController(ros::NodeHandle nodeHandle) 
  : node(std::move(nodeHandle)),
    odomSub(node.subscribe("/odom", 5, &MecanumController::OdomCb, this)),
    colorSub(node.subscribe("/device_0/sensor_1/Color_0/image/data", 5, &MecanumController::ColorImgCb, this)),
    detPub(node.advertise<sensor_msgs::Image>("/color/detection", 5, false)),
    cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
    cmdVelPubRate(10),
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

void MecanumController::OdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
}

void MecanumController::ColorImgCb(const sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  cv::Mat background;
  // cv::cvtColor(img->image, background, cv::COLOR_RGB2HSV);
  cv::cvtColor(img->image, background, cv::COLOR_RGB2Lab);
  // cv::blur(background, background, cv::Size(5, 5), cv::Point(-1, -1));
  cv::GaussianBlur(background, background, cv::Size(5, 5), 1.0, 4.0);
  cv::Mat mask_orange;
  // cv::inRange(background,
  //             cv::Scalar(H_MIN_ORANGE, S_MIN_ORANGE, V_MIN_ORANGE),
  //             cv::Scalar(H_MAX_ORANGE, S_MAX_ORANGE, V_MAX_ORANGE),
  //             mask_orange);
  cv::inRange(background,
              cv::Scalar(L_MIN_ORANGE, A_MIN_ORANGE, B_MIN_ORANGE),
              cv::Scalar(L_MAX_ORANGE, A_MAX_ORANGE, B_MAX_ORANGE),
              mask_orange);
  cv::dilate(mask_orange, mask_orange,
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1,-1)),
             cv::Point(-1, -1), 3);
  // cv::dilate(mask_orange, mask_orange,
  //            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1,-1)),
  //            cv::Point(-1, -1), 1);
  // cv::blur(mask_orange, mask_orange, cv::Size(5, 5), cv::Point(-1, -1));
  // cv::dilate(mask_orange, mask_orange,
  //            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1,-1)),
  //            cv::Point(-1, -1), 1);
  std::vector<std::vector<cv::Point>> orange_contours;
  cv::findContours(mask_orange, orange_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  // cv::imshow("masko", mask_orange);
  // cv::imshow("backg", background);
  // cv::waitKey(1);
  for (const auto& contour : orange_contours) {
    const auto area = cv::contourArea(contour);
    if (area < 5000 or area > 20000) { // TODO: this has to be calibrated
      continue;
    }
    auto bbox = cv::boundingRect(contour);
    cv::rectangle(img->image, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), 
                  cv::Scalar(255, 165, 0), 2, cv::LINE_8, 0);
    cv::putText(img->image, cv::format("Area: %f", area) , cv::Point(bbox.x, bbox.y), 
                cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,255,255), 2);
  }  

  detPub.publish(img->toImageMsg());
}
