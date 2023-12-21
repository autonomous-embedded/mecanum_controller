#include "mecanum_controller.hpp"

#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <utility>
#include <vector>

#define SEND_IMG_MESSAGES 1

struct ObstacleDescription {
  double distanceEstimation;
  int offsetFromCentreX;
  int offsetFromCentreY;
};

enum CtrlCmd {
  FORWARD,
  LEFT,
  RIGHT,
  BACKWARD,
  ROTATE_CLK,
  ROTATE_COUNTER_CLK,
  STOP
};

const char* CtrlCmdMapper(CtrlCmd cmd) {
  switch (cmd) {
    case CtrlCmd::FORWARD: {
      return "FORWARD";
    }
    case CtrlCmd::LEFT: {
      return "LEFT";
    }
    case CtrlCmd::RIGHT: {
      return "RIGHT";
    }
    case CtrlCmd::BACKWARD: {
      return "BACKWARD";
    }
    case CtrlCmd::ROTATE_CLK: {
      return "ROTATE_CLK";
    }
    case CtrlCmd::ROTATE_COUNTER_CLK: {
      return "ROTATE_COUNTER_CLK";
    }
    case CtrlCmd::STOP: {
      return "STOP";
    }
  }
  return "UNKNOWN";
}

MecanumController::MecanumController(ros::NodeHandle nodeHandle) 
  : node(std::move(nodeHandle)),
    odomSub(node.subscribe("/odom", 5, &MecanumController::OdomCb, this)),
    colorSub(node.subscribe("/camera/color/image_raw", 5, &MecanumController::ColorImgCb, this)),
    detPub(node.advertise<sensor_msgs::Image>("/color/detection", 5, false)),
    cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
    cmdVelPubRate(60)
    {}

namespace {
double CalculateLinearControlInRange(const double diff, const double minSpeed, 
                                     const double maxSpeed) {
  double speed = 0.5 * diff;

  if (speed > maxSpeed) {
    speed = maxSpeed;
  }
  else if (speed < minSpeed) {
    speed = minSpeed;
  }
  return speed;
}

std::vector<ObstacleDescription> ProcessObstacles(std::vector<cv::Rect> obstacleList) {
  // make sure we pass the vector by value!
  std::vector<ObstacleDescription> descriptions{obstacleList.size()};

  for (const auto& obstacle : obstacleList) {    
    /* Calculate center of rectangle */
    const int xCenter = static_cast<int>(obstacle.x + (obstacle.width / 2));
    const int yCenter = static_cast<int>(obstacle.y + (obstacle.height / 2));

    /* Calculate area */
    const double area = obstacle.width * obstacle.height;

    /* Add to descriptions */
    ObstacleDescription description;
    description.distanceEstimation = static_cast<double>((double)area / (double)(480 - yCenter));
    description.offsetFromCentreX = xCenter - 320;
    description.offsetFromCentreY = yCenter - 240;
    descriptions.push_back(description);
  }

  return descriptions;
}

geometry_msgs::Twist GetControlCmd(CtrlCmd cmd) {
  geometry_msgs::Twist msg;
  switch (cmd) {
    case CtrlCmd::FORWARD: {
      msg.linear.x = CalculateLinearControlInRange(closestObstacle.distanceEstimation, 0.0, 0.5);
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::LEFT: {
      msg.linear.x = 0.0;
      msg.linear.y = -0.5;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::RIGHT: {
      msg.linear.x = 0.0;
      msg.linear.y = 0.5;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::BACKWARD: {
      msg.linear.x = -0.5;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::ROTATE_CLK: {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.5;
      break;
    }
    case CtrlCmd::ROTATE_COUNTER_CLK: {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = -0.5;
      break;
    }
    case CtrlCmd::STOP: {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
  }
  return msg;
}
} // namespace

void MecanumController::Run() {
  geometry_msgs::Twist msg;

  /* Control logic */

  /* Process the obstacle detections into a more useful structure */
  const auto& processedObstacles = ProcessObstacles(obstacles);

  /* Calculate next control command */
  CtrlCmd cmd{CtrlCmd::STOP};

  ObstacleDescription closestObstacle;
  for (const ObstacleDescription& desc : processedObstacles) {
    if (closestObstacle.distanceEstimation < desc.distanceEstimation) {
      closestObstacle = desc;
    }
  }
  // ROS_DEBUG_COND(closestObstacle.offsetFromCentreX != 0, "Closest obstacle: %f, %d, %d", 
  //   closestObstacle.distanceEstimation, closestObstacle.offsetFromCentreX, closestObstacle.offsetFromCentreY);

  /* X goes (from -320 to 320), Y goes (from -240 to 240) */
  if (closestObstacle.offsetFromCentreX <= -150 || closestObstacle.offsetFromCentreX >= 150) {
    cmd = CtrlCmd::FORWARD;
  }
  else if (closestObstacle.offsetFromCentreX >= 0 && closestObstacle.distanceEstimation <100){
    cmd = CtrlCmd::LEFT;
  }
  else if (closestObstacle.offsetFromCentreX < 0 && closestObstacle.distanceEstimation <100) {
    cmd = CtrlCmd::RIGHT;
  }
  else if (closestObstacle.offsetFromCentreX >= 0 && closestObstacle.distanceEstimation >=100) {
    cmd = CtrlCmd::ROTATE_COUNTER_CLK;
  }
  else if (closestObstacle.offsetFromCentreX < 0 && closestObstacle.distanceEstimation >=100) {
    cmd = CtrlCmd::ROTATE_CLK;
  }
  else {
    cmd = CtrlCmd::STOP;
  }

  msg = GetControlCmd(cmd);
  ROS_DEBUG_THROTTLE(0.1, "Calculated direction: %s", CtrlCmdMapper(cmd));
  
  /* Publish the calculated command */
  cmdVelPub.publish(msg);
  ros::spinOnce();
  cmdVelPubRate.sleep();
}

void MecanumController::OdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
}

void MecanumController::ColorImgCb(const sensor_msgs::ImagePtr& msg) {
  cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
  ROS_DEBUG_THROTTLE(10, "Image width: %d, image height: %d", img.cols, img.rows);
  
  cv::Mat background;
  cv::cvtColor(img, background, cv::COLOR_RGB2Lab);
  cv::GaussianBlur(background, background, cv::Size(5, 5), 1.0, 6.0);
  
  cv::Mat mask_orange;
  cv::inRange(background,
              cv::Scalar(L_MIN_ORANGE, A_MIN_ORANGE, B_MIN_ORANGE),
              cv::Scalar(L_MAX_ORANGE, A_MAX_ORANGE, B_MAX_ORANGE),
              mask_orange);              
  cv::dilate(mask_orange, mask_orange,
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1,-1)),
             cv::Point(-1, -1), 3);

  std::vector<std::vector<cv::Point>> orange_contours;
  cv::findContours(mask_orange, orange_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  {
    std::unique_lock lk{obstacleMutex};
    obstacles.clear();
    cv::Mat bboxMask{background.rows, background.cols, CV_8U};
    for (const auto& contour : orange_contours) {
      const auto area = cv::contourArea(contour);
      if (area < 1000 or area > 10000) { // TODO: this has to be calibrated
        continue;
      }
      auto bbox = cv::boundingRect(contour);
      obstacles.emplace_back(bbox);
#if SEND_IMG_MESSAGES == 1
      cv::rectangle(bboxMask, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), 
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::rectangle(img, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), 
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::putText(img, cv::format("Area: %.2f\nCoords: [x: %d, y:%d]", area, bbox.x, bbox.y),
                  cv::Point(bbox.x, bbox.y), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,255,255), 2);
#endif
    }
  }

#if SEND_IMG_MESSAGES == 1
  cv_bridge::CvImage det_img{msg->header, "rgb8", img};
  detPub.publish(det_img);
#endif
}
