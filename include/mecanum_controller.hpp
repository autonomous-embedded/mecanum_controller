#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <condition_variable>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <variant>
#include <vector>

class MecanumController {
  static constexpr int H_MIN_ORANGE{0}, S_MIN_ORANGE{100}, V_MIN_ORANGE{50};
  static constexpr int H_MAX_ORANGE{30}, S_MAX_ORANGE{255}, V_MAX_ORANGE{255};

  static constexpr int L_MIN_ORANGE{25}, A_MIN_ORANGE{150}, B_MIN_ORANGE{150};
  static constexpr int L_MAX_ORANGE{255}, A_MAX_ORANGE{230}, B_MAX_ORANGE{230};

  static constexpr int IMG_WIDTH{1280}, IMG_HEIGHT{720};

  // node handle
  ros::NodeHandle node;

  // sub
  ros::Subscriber colorSub;

  // pub, timing
  // cmd_vel
  ros::Publisher detPub;
  ros::Publisher cmdVelPub;
  ros::Rate cmdVelPubRate;

  /* vision data */
  std::vector<cv::Rect> obstacles;
  std::mutex obstacleMutex;

 public:
  MecanumController(ros::NodeHandle nodeHandle);

  void Run();

 private:
  void ColorImgCb(const sensor_msgs::ImagePtr& msg);
};

#endif  // MECANUM_CONTROLLER_HPP_
