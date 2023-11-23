#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <ros/ros.h>
// #include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <variant>

// #define DEFAULT_WAIT_TIME_MS 5000 // 5 seconds

// namespace state {
// struct Forward { 
//   double target_distance;
// };

// struct Left {
//   double target_distance;
// };

// struct Right {
//   double target_distance;
// };

// struct Back {
//   double target_distance;
// };

// struct RotateClockwise {
//   double target_angle;
// };

// struct RotateCounterClockwise {
//   double target_angle;
// };

// struct Stop {
//   double waitTimeMs;
// };

// struct EvaluateEnviroment {

// };

// } // namespace state


// enum Direction {
//   FWD = 0,
//   LEFT,
//   RIGHT,
//   BACK
// };

// namespace evt {
// struct StopCmdReceived {
//   double waitTimeMs;
// };

// struct StartCmdReceived { 
//   Direction direction;
//   double distance;
//   double angle;
// };

// struct NoPathFound {
// };

// } // namespace evt

// using State = std::variant<state::Forward, state::Left, state::Right, state::Back,
//                            state::RotateClockwise, state::RotateCounterClockwise,
//                            state::Stop, state::EvaluateEnviroment>;
// using Event = std::variant<evt::StopCmdReceived, evt::StartCmdReceived, evt::NoPathFound>;

// /* Begin state transitions */
// State onEvent(const state::Forward& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::Forward& state, const evt::NoPathFound& evt);

// State onEvent(const state::Left& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::Left& state, const evt::NoPathFound& evt);

// State onEvent(const state::Right& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::Right& state, const evt::NoPathFound& evt);

// State onEvent(const state::Back& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::Back& state, const evt::NoPathFound& evt);

// State onEvent(const state::RotateClockwise& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::RotateClockwise& state, const evt::NoPathFound& evt);

// State onEvent(const state::RotateCounterClockwise& state, const evt::StopCmdReceived& evt);

// State onEvent(const state::RotateCounterClockwise& state, const evt::NoPathFound& evt);

// State onEvent(const state::Stop& state, const evt::StartCmdReceived& evt);
// /* End state transitions */

// struct StateMachine {
//  public:
//   void Enter();
//   void ProcessEvent(const Event& event);

//  private:
//   State state;
// };

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

 public:
  MecanumController(ros::NodeHandle nodeHandle);

  void Run();

 private:
  void OdomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void ColorImgCb(const sensor_msgs::ImagePtr& msg);
};

#endif // MECANUM_CONTROLLER_HPP_