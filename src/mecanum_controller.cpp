#include "mecanum_controller.hpp"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <utility>
#include <vector>

// /* Begin state transitions */
// State onEvent(const state::Forward& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::Forward& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::Left& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::Left& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::Right& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::Right& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::Back& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::Back& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::RotateClockwise& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::RotateClockwise& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::RotateCounterClockwise& state, const evt::StopCmdReceived& evt) {
//   return state::Stop{evt.waitTimeMs};
// }

// State onEvent(const state::RotateCounterClockwise& state, const evt::NoPathFound& evt) {
//   return state::Stop{DEFAULT_WAIT_TIME_MS};
// }

// State onEvent(const state::Stop& state, const evt::StartCmdReceived& evt) {
//   switch (evt.direction) {
//     case FWD: {
//       return state::Forward{evt.distance};
//     }
//     case LEFT: {
//       return state::Left{evt.distance};
//     }
//     case RIGHT: {
//       return state::Right{evt.distance};
//     }
//     case BACK: {
//       return state::Back{evt.distance};
//     }
//     default: {
//       return state::EvaluateEnviroment{};
//     }
//   }
// }
// /* End state transitions */

// void StateMachine::Enter() {
//   state = state::EvaluateEnviroment{};
// }

// void StateMachine::ProcessEvent(const Event& event) {
//   state = std::visit([](const auto& state, const auto& evt) {
//       return onEvent(state, evt);
//   }, state, event);
// }

MecanumController::MecanumController(ros::NodeHandle nodeHandle) 
  : node(std::move(nodeHandle)),
    odomSub(node.subscribe("/odom", 5, &MecanumController::OdomCb, this)),
    colorSub(node.subscribe("/device_0/sensor_1/Color_0/image/data", 5, &MecanumController::ColorImgCb, this)),
    detPub(node.advertise<sensor_msgs::Image>("/color/detection", 5, false)),
    cmdVelPub(node.advertise<geometry_msgs::Twist>("/cmd_vel", 5, false)),
    cmdVelPubRate(10)
    {}

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
  geometry_msgs::Twist msg;

  /* Control logic */

  /* Process the obstacle detections into a more useful structure */
  // const auto& processedObstacles = ProcessObstacles();

  /* Calculate what path we can take */
  // const auto& path = CalculatePath();

  /* Calculate next control command, either:
   *  - forward
   *  - left
   *  - right
   *  - stop (if we can't move anywhere)
   */
  // msg = GetControlCmd();
  
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
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  cv::Mat background;
  // cv::cvtColor(img->image, background, cv::COLOR_RGB2HSV);
  cv::cvtColor(img->image, background, cv::COLOR_RGB2Lab);
  cv::GaussianBlur(background, background, cv::Size(5, 5), 1.0, 6.0);
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
  cv::imshow("backg", background);
  cv::waitKey(1);

  {
    std::unique_lock lk{obstacleMutex};
    obstacles.clear();
    cv::Mat bboxMask{background.rows, background.cols, CV_8U};
    for (const auto& contour : orange_contours) {
      const auto area = cv::contourArea(contour);
      if (area < 10000 or area > 40000) { // TODO: this has to be calibrated
        continue;
      }
      auto bbox = cv::boundingRect(contour);
      obstacles.emplace_back(bbox);

      cv::rectangle(bboxMask, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), 
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::Scalar mean = cv::mean(background, bboxMask);
      const auto LMean = mean.val[0];
      const auto aMean = mean.val[1];
      const auto bMean = mean.val[2];

      cv::rectangle(img->image, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), 
                    cv::Scalar(255), 2, cv::LINE_8, 0);
      cv::putText(img->image, cv::format("Area: %f\n[L,a,b] average: [%lf, %lf, %lf]", 
                              area, LMean, aMean, bMean),
                  cv::Point(bbox.x, bbox.y), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,255,255), 2);
    }
  }

  detPub.publish(img->toImageMsg());
}
