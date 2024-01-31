#include "mecanum_controller.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

#define SEND_IMG_MESSAGES 1

struct ObstacleDescription {
  ObstacleDescription()
      : distanceEstimation{0.0}, offsetFromCentreX{0}, offsetFromCentreY{0} {}

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

const char* CtrlCmdStringMapper(const CtrlCmd cmd) {
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
      colorSub(node.subscribe("/camera/color/image_raw", 5,
                              &MecanumController::ColorImgCb, this)),
      detPub(node.advertise<sensor_msgs::Image>(
          "/mecanum_controller/color/detection", 5, false)),
      cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
      cmdVelPubRate(4) {}

namespace {
double CalculateLinearControlInRange(const double diff, const double minSpeed,
                                     const double maxSpeed) {
  double speed = 0.5 * diff;

  if (diff > maxSpeed) {
    speed = maxSpeed;
  } else if (diff < minSpeed) {
    speed = minSpeed;
  }

  return speed;
}

std::vector<ObstacleDescription> ProcessObstacles(
    const std::vector<cv::Rect> obstacleList, const int imgWidth,
    const int imgHeight) {
  std::vector<ObstacleDescription> descriptions{obstacleList.size()};

  for (const auto& obstacle : obstacleList) {
    /* Calculate center of rectangle */
    const int xCenter = static_cast<int>(obstacle.x + (obstacle.width / 2));
    const int yCenter = static_cast<int>(obstacle.y + (obstacle.height / 2));

    /* Calculate area */
    const double area = obstacle.width * obstacle.height;

    /* Add to descriptions */
    ObstacleDescription description;
    description.distanceEstimation =
        static_cast<double>(area) / static_cast<double>(imgHeight - yCenter);
    description.offsetFromCentreX = xCenter - imgWidth / 2;
    description.offsetFromCentreY = yCenter - imgHeight / 2;
    descriptions.push_back(description);
  }

  return descriptions;
}

const geometry_msgs::Twist GetControlCmd(CtrlCmd cmd) {
  geometry_msgs::Twist msg;
  switch (cmd) {
    case CtrlCmd::FORWARD: {
      msg.linear.x = 0.3;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::LEFT: {
      msg.linear.x = 0.3;
      msg.linear.y = -0.3;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::RIGHT: {
      msg.linear.x = 0.3;
      msg.linear.y = 0.3;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      break;
    }
    case CtrlCmd::BACKWARD: {
      msg.linear.x = -0.3;
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
      msg.linear.x = 0.5;
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

const CtrlCmd CalculateControl(const ObstacleDescription& closestObstacle,
                               const ObstacleDescription& secondClosestObstacle,
                               const int imgWidth, const int imgHeight) {
  CtrlCmd cmd{CtrlCmd::STOP};

  /* If closest and second closest are the same, just stop the car */
  if ((closestObstacle.distanceEstimation == 0.0) &&
      (secondClosestObstacle.distanceEstimation == 0.0)) {
    return CtrlCmd::STOP;
  }

  /* General comment - 1 is closest, 2 is further */
  bool isClosestLeft = false;
  if (closestObstacle.offsetFromCentreX <
      secondClosestObstacle.offsetFromCentreX) {
    isClosestLeft = true;
  } else {
    isClosestLeft = false;
  }

  double x1 = static_cast<double>(closestObstacle.offsetFromCentreX);
  double x2 = static_cast<double>(secondClosestObstacle.offsetFromCentreX);

  /* Normalize */
  const double xMax = std::max(x1, x2);
  x1 /= xMax;
  x2 /= xMax;
  ROS_INFO("x1: %lf, x2: %lf", x1, x2);

  const double distanceMax = std::max(closestObstacle.distanceEstimation,
                                      secondClosestObstacle.distanceEstimation);
  const double d1 = closestObstacle.distanceEstimation / distanceMax;
  const double d2 = secondClosestObstacle.distanceEstimation / distanceMax;
  ROS_INFO("d1: %lf, d2: %lf", d1, d2);

  /* Calculate weights using distance and x offset */
  const double w1 = x1 * d1;
  const double w2 = x2 * d2;
  ROS_INFO("Weight to first: %lf, weight to second: %lf", w1, w2);

  const double weigth = (isClosestLeft ? (w1 - w2) : (w2 - w1));
  ROS_INFO("Final Weight: %lf", weigth);
  /* Condition for stopping the car:
   *  - calculated weights both under ??? (todo->value)
   */
  if (weigth > 0.0) {
    cmd = CtrlCmd::STOP;
  }

  return cmd;
}
}  // namespace

void MecanumController::Run() {
  /* Control logic */
  geometry_msgs::Twist msg;

  /* Process the obstacle detections into a more useful structure */
  const auto processedObstacles =
      ProcessObstacles(obstacles, IMG_WIDTH, IMG_HEIGHT);

  /* Find obstacle closest to the car */
  ObstacleDescription closestObstacle;
  ObstacleDescription secondClosestObstacle;
  for (const ObstacleDescription& desc : processedObstacles) {
    if (closestObstacle.distanceEstimation < desc.distanceEstimation) {
      secondClosestObstacle = closestObstacle;
      closestObstacle = desc;
    }
  }

  /* Determine where the car should drive */
  CtrlCmd cmd{CalculateControl(closestObstacle, secondClosestObstacle,
                               IMG_WIDTH, IMG_HEIGHT)};

  /* Get message for the calculated direction */
  msg = GetControlCmd(cmd);
  ROS_INFO("Control Command: %s", CtrlCmdStringMapper(cmd));

  /* Publish the calculated command */
  cmdVelPub.publish(msg);

  ros::spinOnce();
  cmdVelPubRate.sleep();
}

void MecanumController::ColorImgCb(const sensor_msgs::ImagePtr& msg) {
  cv::Mat img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;

  ROS_ASSERT_MSG((img.cols == IMG_WIDTH) && (img.rows == IMG_HEIGHT),
                 "Unexpected image size");

  cv::Mat background;
  cv::cvtColor(img, background, cv::COLOR_RGB2Lab);
  cv::GaussianBlur(background, background, cv::Size(5, 5), 1.0, 6.0);

  cv::Mat maskOrange;
  cv::inRange(background, cv::Scalar(L_MIN_ORANGE, A_MIN_ORANGE, B_MIN_ORANGE),
              cv::Scalar(L_MAX_ORANGE, A_MAX_ORANGE, B_MAX_ORANGE), maskOrange);
  cv::dilate(maskOrange, maskOrange,
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5),
                                       cv::Point(-1, -1)),
             cv::Point(-1, -1), 3);

  // guess something more than zero, so we don't waste time in malloc
  std::vector<std::vector<cv::Point>> orangeContours{5};
  cv::findContours(maskOrange, orangeContours, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);
  {
    std::unique_lock<std::mutex> lk(obstacleMutex);
    obstacles.clear();
    cv::Mat bboxMask{IMG_HEIGHT, IMG_WIDTH, CV_8U};

    for (const auto& contour : orangeContours) {
      const auto area = cv::contourArea(contour);
      const auto bbox = cv::boundingRect(contour);
      obstacles.emplace_back(bbox);
      cv::rectangle(bboxMask, cv::Point(bbox.x, bbox.y),
                    cv::Point(bbox.x + bbox.width, bbox.y + bbox.height),
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::rectangle(img, cv::Point(bbox.x, bbox.y),
                    cv::Point(bbox.x + bbox.width, bbox.y + bbox.height),
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::putText(
          img,
          cv::format("Area: %.2f\nCoords: [x: %d, y:%d]", area, bbox.x, bbox.y),
          cv::Point(bbox.x, bbox.y), cv::FONT_HERSHEY_DUPLEX, 1.0,
          CV_RGB(255, 255, 255), 2);
    }
  }

  const cv_bridge::CvImage detImg{msg->header, "rgb8", img};
  detPub.publish(detImg);
}
