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

#ifndef TURTLESIM__TURTLE_FRAME_HPP_
#define TURTLESIM__TURTLE_FRAME_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "my_turtle.hpp"  // NO LINT
#endif

#include "my_turtlesim/agv_modbus_mapping.hpp"
#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>
#include <QLineEdit>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
#include <my_turtlesim_msgs/msg/spawn_request.hpp>
//#include <my_turtlesim_msgs/srv/kill.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_turtlesim_msgs/msg/spawn_request.hpp>


#endif

namespace my_turtlesim
{
class TurtleFrame : public QFrame
{
  Q_OBJECT

public:
  TurtleFrame(
    rclcpp::Node::SharedPtr & node_handle, QWidget * parent = 0,
    Qt::WindowFlags f = Qt::WindowFlags());
  ~TurtleFrame();

  std::string spawnTurtle(const std::string & name, float x, float y, float angle);
  std::string spawnTurtle(const std::string & name, float x, float y, float angle, size_t index);



signals:
    void requestGuiUpdate();

protected:
  void paintEvent(QPaintEvent * event);
  void resizeEvent(QResizeEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
private:
    // 拖动相关成员
    bool dragging_ = false;
    std::string dragging_turtle_name_;
    QPoint drag_offset_;
    QVector<QLineEdit*> turtle_coord_edits_; // 新增：每个按钮下的文本框
    const int button_count = 8; // 放到头文件或全局

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string & name);

  bool clearCallback(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);
  bool resetCallback(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);

  void checkTurtleCollisions();
  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread ros_spin_thread_;


  rclcpp::Node::SharedPtr nh_;

  QTimer * update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  rclcpp::Time last_turtle_update_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  // rclcpp::Service<my_turtlesim_msgs::srv::Spawn>::SharedPtr spawn_srv_;
  // rclcpp::Service<my_turtlesim_msgs::srv::Kill>::SharedPtr kill_srv_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kill_sub_; // 新增，kill话题订阅

  void killTopicCallback(const std_msgs::msg::String::SharedPtr msg); // 新增，kill话题回调

  // 头文件加
  rclcpp::Subscription<my_turtlesim_msgs::msg::SpawnRequest>::SharedPtr spawn_sub_;
  void spawnTopicCallback(const my_turtlesim_msgs::msg::SpawnRequest::SharedPtr msg);

  void loadConfigFile(const QString& filename);

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;

  QVector<QRect> button_rects_;
  QVector<bool> button_states_;   // 按钮状态（true=已生成，false=未生成）
  QRect save_button_rect_;
  QRect load_button_rect_;
  int next_modbus_port_ = 1502; // 成员变量

  std::string local_id;
};
}  // namespace my_turtlesim
#endif  // TURTLESIM__TURTLE_FRAME_HPP_
