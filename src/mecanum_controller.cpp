#include "mecanum_controller.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

#include <ctime>
#include <opencv2/opencv.hpp>
#include <optional>
#include <utility>
#include <vector>

#define SEND_IMG_MESSAGES 1

struct ObstacleDescription {
  ObstacleDescription()
      : distanceEstimation{0.0},
        offsetFromCentreX{0},
        offsetFromCentreY{0},
        idx{-1} {}

  double distanceEstimation;
  int offsetFromCentreX;
  int offsetFromCentreY;
  int idx;
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
      colorSub(node.subscribe("/camera/color/image_raw", 8,
                              &MecanumController::ColorImgCb, this)),
      detPub(node.advertise<sensor_msgs::Image>(
          "/mecanum_controller/color/detection", 5, false)),
      cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
      cmdVelPubRate(4) {}

namespace {
std::vector<ObstacleDescription> ProcessObstacles(
    const std::vector<cv::Rect> obstacleList, const int imgWidth,
    const int imgHeight) {
  std::vector<ObstacleDescription> descriptions{obstacleList.size()};

  int idx = 0;
  for (const auto& obstacle : obstacleList) {
    /* Calculate center of rectangle */
    const int xCenter = static_cast<int>(obstacle.x + (obstacle.width / 2));
    const int yCenter = static_cast<int>(obstacle.y + (obstacle.height / 2));

    /* Calculate area */
    const double area = (double)obstacle.width * (double)obstacle.height;

    /* Add to descriptions */
    ObstacleDescription description;
    description.distanceEstimation =
        (static_cast<double>(area) - 3'000.0) / 97'000.0;  // ugh...
    description.offsetFromCentreX = xCenter - imgWidth / 2;
    description.offsetFromCentreY = yCenter - imgHeight / 2;
    description.idx = idx;
    descriptions.push_back(description);

    idx += 1;
  }

  std::sort(std::begin(descriptions), std::end(descriptions),
            [](const auto& d1, const auto& d2) {
              return d1.distanceEstimation < d2.distanceEstimation;
            });

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
      msg.angular.z = -0.2;
      break;
    }
    case CtrlCmd::ROTATE_COUNTER_CLK: {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.2;
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

static constexpr double FWD_THRESH{0.15};
static constexpr double BCK_THRESH{0.95};

const CtrlCmd CalculateControlForOneObstacle(
    const ObstacleDescription& closestObstacle) {
  static bool rotatingClockwise = false;
  CtrlCmd cmd{CtrlCmd::STOP};

  /* Stop in front of obstacle if too close */
  const bool isInFront = std::abs(closestObstacle.offsetFromCentreX) < 50;
  const bool isTooClose = closestObstacle.distanceEstimation > BCK_THRESH;
  if (isInFront && isTooClose) {
    cmd = CtrlCmd::BACKWARD;
  }
  /* If stuff is far away, just drive forwards */
  else if (closestObstacle.distanceEstimation < FWD_THRESH) {
    cmd = CtrlCmd::FORWARD;
  }
  /* Else rotate either left or right, depending on the offset from the center
   * This has to be implemented to be stateful...*/
  else {
    if (rotatingClockwise) {
      cmd = CtrlCmd::ROTATE_CLK;
      /* If we see the object at the far left, change the rotation mode to
       * counterclockwise */
      if (closestObstacle.offsetFromCentreX < -300) {
        rotatingClockwise = false;
      }
    } else {
      cmd = CtrlCmd::ROTATE_COUNTER_CLK;
      /* If we see the object at the far right, change the rotation mode to
       * clockwise */
      if (closestObstacle.offsetFromCentreX > 300) {
        rotatingClockwise = true;
      }
    }
  }
  ROS_INFO("distance: %.2lf, offset: %d", closestObstacle.distanceEstimation,
           closestObstacle.offsetFromCentreX);

  return cmd;
}

const CtrlCmd CalculateControlForTwoObstacles(
    const ObstacleDescription& closestObstacle,
    const ObstacleDescription& secondClosestObstacle) {
  CtrlCmd cmd{CtrlCmd::STOP};

  /* Determine if either is too close */
  const bool areTooClose =
      ((closestObstacle.distanceEstimation > BCK_THRESH) ||
       (secondClosestObstacle.distanceEstimation > BCK_THRESH));
  if (areTooClose) {
    cmd = CtrlCmd::BACKWARD;
  }
  /* If both are far away, drive forward */
  else if ((closestObstacle.distanceEstimation) <= FWD_THRESH &&
           (secondClosestObstacle.distanceEstimation <= FWD_THRESH)) {
    cmd = CtrlCmd::FORWARD;
    ROS_INFO("closest dist: %.2lf, second closest: %.2lf",
             closestObstacle.distanceEstimation,
             secondClosestObstacle.distanceEstimation);
  }
  /* Determine where is the middle between obstacles */
  else {
    const ObstacleDescription& leftmost =
        (closestObstacle.offsetFromCentreX <
         secondClosestObstacle.offsetFromCentreX)
            ? closestObstacle
            : secondClosestObstacle;
    const ObstacleDescription& rightmost =
        (closestObstacle.offsetFromCentreX >
         secondClosestObstacle.offsetFromCentreX)
            ? closestObstacle
            : secondClosestObstacle;

    const int midpoint = std::abs(rightmost.offsetFromCentreX) -
                         std::abs(leftmost.offsetFromCentreX);

    if (rightmost.offsetFromCentreX < -200) {
      cmd = CtrlCmd::ROTATE_COUNTER_CLK;
    } else if (leftmost.offsetFromCentreX > 200) {
      cmd = CtrlCmd::ROTATE_CLK;
    } else {
      if (midpoint < -250) {
        cmd = CtrlCmd::LEFT;
      } else if (midpoint > 250) {
        cmd = CtrlCmd::RIGHT;
      } else {
        cmd = CtrlCmd::FORWARD;
      }
    }
    ROS_INFO("closest dist: %.2lf, second closest: %.2lf, midpoint: %d",
             closestObstacle.distanceEstimation,
             secondClosestObstacle.distanceEstimation, midpoint);
  }

  return cmd;
}

const CtrlCmd CalculateControl(
    const std::optional<ObstacleDescription>& closestObstacleOpt,
    const std::optional<ObstacleDescription>& secondClosestObstacleOpt,
    const int imgWidth, const int imgHeight) {
  CtrlCmd cmd{CtrlCmd::STOP};
  static bool searching = true;

  if (searching) {
    searching = true;

    /* none found yet */
    if ((!closestObstacleOpt.has_value()) &&
        (!secondClosestObstacleOpt.has_value())) {
      cmd = CtrlCmd::ROTATE_CLK;
    } else {
      searching = false;
      cmd = CtrlCmd::STOP;
    }

    return cmd;
  }

  /* If not searching */
  if ((!closestObstacleOpt.has_value()) &&
      (!secondClosestObstacleOpt.has_value())) {
    /* Maybe we still should... */
    searching = true;
    return CtrlCmd::STOP;
  }

  /* Algorithm 1: Only one obstacle is visible or
   * difference the distance between two obstacles
   * is too big to be able say that they form a gate */
  if ((closestObstacleOpt.has_value()) &&
      ((!secondClosestObstacleOpt.has_value()) ||
       (secondClosestObstacleOpt.value().distanceEstimation == 0.0))) {
    cmd = CalculateControlForOneObstacle(closestObstacleOpt.value());
  }
  /* Algorithm 2: Both obstacles visible */
  else {
    cmd = CalculateControlForTwoObstacles(closestObstacleOpt.value(),
                                          secondClosestObstacleOpt.value());
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

  /* Determine where the car should drive */
  std::size_t numOfObstacles{processedObstacles.size()};
  CtrlCmd cmd{CalculateControl(
      (numOfObstacles > 0
           ? std::optional<
                 ObstacleDescription>{processedObstacles[numOfObstacles - 1]}
           : std::nullopt),
      (numOfObstacles > 1
           ? std::optional<
                 ObstacleDescription>{processedObstacles[numOfObstacles - 2]}
           : std::nullopt),
      IMG_WIDTH, IMG_HEIGHT)};

  /* Get message for the calculated direction */
  msg = GetControlCmd(cmd);
  ROS_INFO("Control Command: %s", CtrlCmdStringMapper(cmd));

  /* Publish the calculated command */
  cmdVelPub.publish(msg);

  ros::spinOnce();
  cmdVelPubRate.sleep();
}

/* CRITICAL SECTION - be quick here...
 * traces should be left, it's OK to go as slow as 500ms IMO. */
static time_t currentTime;
void MecanumController::ColorImgCb(const sensor_msgs::ImagePtr& msg) {
  std::time(&currentTime);
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

  // Guess something more than zero, so we don't waste time in malloc
  std::vector<std::vector<cv::Point>> orangeContours{5};
  cv::findContours(maskOrange, orangeContours, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);
  {
    std::unique_lock<std::mutex> lk(obstacleMutex);
    obstacles.clear();
    cv::Mat bboxMask{IMG_HEIGHT, IMG_WIDTH, CV_8U};

    for (const auto& contour : orangeContours) {
      const auto bbox = cv::boundingRect(contour);
      const auto area = bbox.width * bbox.height;
      if ((area > 3'000) && (area < 100'000)) {
        obstacles.emplace_back(bbox);
      }
      cv::rectangle(bboxMask, cv::Point(bbox.x, bbox.y),
                    cv::Point(bbox.x + bbox.width, bbox.y + bbox.height),
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::rectangle(img, cv::Point(bbox.x, bbox.y),
                    cv::Point(bbox.x + bbox.width, bbox.y + bbox.height),
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::putText(
          img,
          cv::format("Area: %d\nCoords: [x: %d, y:%d]", area, bbox.x, bbox.y),
          cv::Point(bbox.x, bbox.y), cv::FONT_HERSHEY_DUPLEX, 1.0,
          CV_RGB(255, 255, 255), 2);
    }
  }

  const cv_bridge::CvImage detImg{msg->header, "rgb8", img};
  detPub.publish(detImg);

  time_t newTime;
  std::time(&newTime);
  ROS_INFO_THROTTLE_NAMED(5, "ImageProcessingTrace",
                          "MecanumController::ColorImgCb: %.06lf [s]",
                          std::difftime(newTime, currentTime));
}
