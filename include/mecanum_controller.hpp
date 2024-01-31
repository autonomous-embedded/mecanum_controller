#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
// #include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <variant>
#include <mutex>
#include <condition_variable>

#include <opencv2/core/types.hpp>

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

  // state
  double posX;
  double posY;

  // dynamic img size 
  int img_width;
  int img_height;
  std::vector<cv::Rect> obstacles;
  std::mutex obstacleMutex;
  // std::obstacleCv;

 public:
  MecanumController(ros::NodeHandle nodeHandle);

  void Run();

 private:
  void OdomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void ColorImgCb(const sensor_msgs::ImagePtr& msg);
};

#endif // MECANUM_CONTROLLER_HPP_
