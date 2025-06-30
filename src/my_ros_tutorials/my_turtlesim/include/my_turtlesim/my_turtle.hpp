// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TURTLESIM__TURTLE_HPP_
#define TURTLESIM__TURTLE_HPP_

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <rclcpp_action/rclcpp_action.hpp>
// 添加头文件
#include <modbus/modbus.h>

# include <geometry_msgs/msg/twist.hpp>
# include <my_turtlesim_msgs/action/rotate_absolute.hpp>
# include <my_turtlesim_msgs/action/walk_absolute.hpp>
# include <my_turtlesim_msgs/msg/color.hpp>
# include <my_turtlesim_msgs/msg/pose.hpp>
# include <my_turtlesim_msgs/srv/set_pen.hpp>
# include <my_turtlesim_msgs/srv/teleport_absolute.hpp>
# include <my_turtlesim_msgs/srv/teleport_relative.hpp>
# include <my_turtlesim/agv_modbus_mapping.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QElapsedTimer>

#include <memory>
#include <string>
#include <vector>

#define PI 3.14159265
#define TWO_PI 2.0 * PI

namespace my_turtlesim
{
class Turtle
{
public:
  using RotateAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<
    my_turtlesim_msgs::action::RotateAbsolute>;

  using WalkAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<
    my_turtlesim_msgs::action::WalkAbsolute>;

  Turtle(
    rclcpp::Node::SharedPtr & nh, const std::string & real_name, const QImage & turtle_image,
    const QPointF & pos, float orient, int modbus_port, bool delay_modbus);
  ~Turtle(); // 显式声明析构函数
  bool update(
    double dt, QPainter & path_painter, const QImage & path_image, qreal canvas_width,
    qreal canvas_height);
  void paint(QPainter & painter);

  void process_modbus_requests();
  void init_modbus();
  void setOrient(double angle);

  static double pointToLineSignedDistance(
    const QPointF& point,
    double goal_x,
    double goal_y,
    double goal_theta_deg);

  rclcpp::Logger getLogger() const { return nh_->get_logger(); }
  QPointF getPos() const { return pos_; }
  qreal getOrient() const { return orient_; }

  qreal goal_vel;

  RisingEdgeDetector Agv_Start_detector;
  
  void syn_agv_para();

  double HH=5.0;
  QPointF point_f;
  QPointF point_b;

  QPointF point_l;
  QPointF point_r;

  double ang_f;
  double ang_b;
  double ang_m;
  double now_R;

  double goal_x_ = 0;
  double goal_y_ = 0;
  double goal_theta_ = 0;


public:
    void setPos(double x, double y) { pos_.setX(x); pos_.setY(y); }
    bool highlight_error_ = false;


private:
  modbus_t* modbus_ctx_ = nullptr;
  modbus_mapping_t* modbus_mapping_ = nullptr;
  int modbus_server_socket_fd_ = -1;
  int modbus_client_socket_fd_ = -1;


  DB51_TCP db51_tcp_{}; // 新增：DB51_TCP 结构体实例
  DB50_Agv db50_agv_{}; // 新增：DB50_AGV 结构体实例

  void agv_state_machine(); // 声明

  void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel);
  bool setPenCallback(
    const my_turtlesim_msgs::srv::SetPen::Request::SharedPtr,
    my_turtlesim_msgs::srv::SetPen::Response::SharedPtr);
  bool teleportRelativeCallback(
    const my_turtlesim_msgs::srv::TeleportRelative::Request::SharedPtr,
    my_turtlesim_msgs::srv::TeleportRelative::Response::SharedPtr);
  bool teleportAbsoluteCallback(
    const my_turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr,
    my_turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr);
  void rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle>);

  //自己加的
  void walkAbsoluteAcceptCallback(const std::shared_ptr<WalkAbsoluteGoalHandle>);

  void rotateImage();


  QElapsedTimer error_timer_;

  rclcpp::Node::SharedPtr nh_;
  std::string real_name; // 新增：保存乌龟名字
  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;
  int modbus_port_;

  qreal lin_vel_x_;
  qreal lin_vel_y_;
  qreal ang_vel_;
  bool pen_on_;
  QPen pen_;




  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<my_turtlesim_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<my_turtlesim_msgs::msg::Color>::SharedPtr color_pub_;
  rclcpp::Service<my_turtlesim_msgs::srv::SetPen>::SharedPtr set_pen_srv_;
  rclcpp::Service<my_turtlesim_msgs::srv::TeleportRelative>::SharedPtr teleport_relative_srv_;
  rclcpp::Service<my_turtlesim_msgs::srv::TeleportAbsolute>::SharedPtr teleport_absolute_srv_;

  rclcpp_action::Server<my_turtlesim_msgs::action::RotateAbsolute>
  ::SharedPtr rotate_absolute_action_server_;
  std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
  std::shared_ptr<my_turtlesim_msgs::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
  std::shared_ptr<my_turtlesim_msgs::action::RotateAbsolute::Result> rotate_absolute_result_;
  qreal rotate_absolute_start_orient_;


  //仿照旋转写的绝对值导航-开始
  rclcpp_action::Server<my_turtlesim_msgs::action::WalkAbsolute>::SharedPtr 
  walk_absolute_action_server_;
  std::shared_ptr<WalkAbsoluteGoalHandle> 
  server_walk_absolute_goal_handle_;
  std::shared_ptr<my_turtlesim_msgs::action::WalkAbsolute::Feedback> 
  server_walk_absolute_feedback_;
  std::shared_ptr<my_turtlesim_msgs::action::WalkAbsolute::Result> 
  server_walk_absolute_result_;
  qreal server_walk_absolute_start_orient_;
  //仿照旋转写的绝对值导航-结束

  std::shared_ptr<rclcpp_action::Client<my_turtlesim_msgs::action::WalkAbsolute>> 
  walk_absolute_action_client_;
  rclcpp_action::ClientGoalHandle<my_turtlesim_msgs::action::WalkAbsolute>::SharedPtr 
  client_walk_absolute_goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<my_turtlesim_msgs::action::WalkAbsolute>::SharedPtr> 
  client_walk_absolute_goal_handle_future_;


  rclcpp::Time last_command_time_;

  float meter_;

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
    : pos(x, y),
      theta(_theta),
      linear(_linear),
      relative(_relative)
    {
    }

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef std::shared_ptr<Turtle> TurtlePtr;




}  // namespace my_turtlesim

#endif  // TURTLESIM__TURTLE_HPP_
