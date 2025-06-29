#include "my_turtlesim/my_turtle_frame.hpp"

#include <QPointF>
#include <QTime>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <QApplication>    // 新增
#include <QPalette>        // 新增

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

//#include "my_turtlesim_msgs/srv/kill.hpp"
#include "my_turtlesim_msgs/msg/spawn_request.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace my_turtlesim
{

void TurtleFrame::resizeEvent(QResizeEvent *event)
{
  // 重新设置 path_image_ 大小
  if (path_painter_.isActive()) {
      path_painter_.end();
  }
  path_image_ = QImage(event->size(), QImage::Format_ARGB32);
  path_image_.fill(qRgba(255, 255, 255, 0));
  // 不要在这里 begin，等需要绘制路径时再 begin
  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;

  update();
  QFrame::resizeEvent(event);
}

void TurtleFrame::mousePressEvent(QMouseEvent *event)
{
    for (int i = 0; i < button_rects_.size(); ++i) {
        if (button_rects_[i].contains(event->pos())) {
            QString name = QString("No%1").arg(i);
            if (!button_states_[i]) {
                auto pub = nh_->create_publisher<my_turtlesim_msgs::msg::SpawnRequest>("/spawn"+local_id, 10);
                my_turtlesim_msgs::msg::SpawnRequest msg;
                msg.x = 20.0;
                msg.y = 20.0;
                msg.theta = 45.0;
                msg.name = name.toStdString();
                pub->publish(msg);

            } else {

                auto pub = nh_->create_publisher<std_msgs::msg::String>("/kill"+local_id, 10);
                std_msgs::msg::String msg;
                msg.data = name.toStdString();
                pub->publish(msg);
            }
                        // 立即刷新按钮状态和界面
            updateTurtles();
            update();
            break;
        }
    }
    QFrame::mousePressEvent(event);
}

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(800, 600, QImage::Format_ARGB32)
  //, path_painter_(&path_image_)//这里改了，不要传 &path_image_，让它初始未激活
  //不然会在控制台打印错误消息
  , path_painter_()
  , frame_count_(0)
  , id_counter_(0)
{

  local_id = get_local_id_from_ip(); 
  setWindowTitle("乌龟控制");



    // —— 在这里加入 —— 
  setAutoFillBackground(true);
  {
    QPalette pal = palette();
    // 使用系统控件背景色，也可以改成 QPalette::Window
    //pal.setColor(QPalette::Window, QApplication::palette().color(QPalette::button));

    pal.setColor(QPalette::Window, QColor(180, 180, 180));
    setPalette(pal);
  }
  // —— 加入结束 —— 


  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;


  connect(this, &TurtleFrame::requestGuiUpdate, this, [this]() {
      updateTurtles();
      update();
  }, Qt::QueuedConnection);

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;
  rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
  background_r_descriptor.description = "Red channel of the background color";
  background_r_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
  background_g_descriptor.description = "Green channel of the background color";
  background_g_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
  background_b_descriptor.description = "Blue channel of the background color";
  background_b_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
    "background_r", rclcpp::ParameterValue(
      DEFAULT_BG_R), background_r_descriptor);
  nh_->declare_parameter(
    "background_g", rclcpp::ParameterValue(
      DEFAULT_BG_G), background_g_descriptor);
  nh_->declare_parameter(
    "background_b", rclcpp::ParameterValue(
      DEFAULT_BG_B), background_b_descriptor);

  rcl_interfaces::msg::ParameterDescriptor holonomic_descriptor;
  holonomic_descriptor.description = "If true, then turtles will be holonomic";
  nh_->declare_parameter("holonomic", rclcpp::ParameterValue(false), holonomic_descriptor);

  QVector<QString> turtles;
  // turtles.append("ardent.png");
  // turtles.append("bouncy.png");
  // turtles.append("crystal.png");
  // turtles.append("dashing.png");
  // turtles.append("eloquent.png");
  // turtles.append("foxy.png");
  // turtles.append("galactic.png");
  // turtles.append("humble.png");
  // turtles.append("iron.png");
  // turtles.append("jazzy.png");
  // turtles.append("kilted.png");
  // turtles.append("rolling.png");

  turtles.append("No0.png");
  turtles.append("No1.png");
  turtles.append("No2.png");
  turtles.append("No3.png");
  turtles.append("No4.png");
  turtles.append("No5.png");

  turtles.append("No6.png");
  turtles.append("No7.png");
  turtles.append("No8.png");
  turtles.append("No9.png");

  turtles.append("kilted.png");
  turtles.append("rolling.png");



  QString images_path =
    (ament_index_cpp::get_package_share_directory("my_turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i) {
      QImage img;
      QString img_path = images_path + turtles[i];
      bool loaded = img.load(img_path);
      if (!loaded) {
          RCLCPP_ERROR(nh_->get_logger(), "Failed to load image: %s", img_path.toStdString().c_str());
      }
      turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height()/30.0;//乌龟的高度就是1m

  clear();

  clear_srv_ =
    nh_->create_service<std_srvs::srv::Empty>(
    "clear"+local_id,
    std::bind(&TurtleFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ =
    nh_->create_service<std_srvs::srv::Empty>(
    "reset"+local_id,
    std::bind(&TurtleFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

  spawn_sub_ = nh_->create_subscription<my_turtlesim_msgs::msg::SpawnRequest>(
    "spawn"+local_id,
    10,
    std::bind(&TurtleFrame::spawnTopicCallback, this, std::placeholders::_1));

  kill_sub_ = nh_->create_subscription<std_msgs::msg::String>(
    "kill"+local_id,
    10, //订阅队列最多有10条待处理信息
    std::bind(&TurtleFrame::killTopicCallback, this, std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos,
    std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    nh_->get_logger(), "Starting my_turtlesim with node name %s", nh_->get_fully_qualified_name());

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  //spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  //修改为画布的起始位置
  //spawnTurtle("", 0, 0, 0);

  // spawn all available turtle types
  if (false) {
    for (int index = 0; index < turtles.size(); ++index) {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(
        name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7),
        static_cast<float>(PI) / 2.0f, index);
    }
  }


}

TurtleFrame::~TurtleFrame()
{
  // executor_->cancel();
  // if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

  delete update_timer_;
}



void TurtleFrame::spawnTopicCallback(const my_turtlesim_msgs::msg::SpawnRequest::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(), "spawnTopicCallback start");
    std::string name = spawnTurtle(msg->name, msg->x, msg->y, msg->theta/180.0*PI);
    if (name.empty()) {
      RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", msg->name.c_str());
      return;
    }
    //emit requestGuiUpdate();

RCLCPP_INFO(nh_->get_logger(), "spawnTopicCallback end");



}







void TurtleFrame::killTopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string name = msg->data;
    auto it = turtles_.find(name);
    if (it != turtles_.end()) {
        turtles_.erase(it);
        update();
        emit requestGuiUpdate();
    }
    else {
        RCLCPP_ERROR(nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", name.c_str());
    }
}



void TurtleFrame::parameterEventCallback(
  const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name()) {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool TurtleFrame::hasTurtle(const std::string & name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string & name, float x, float y, float angle)
{
  //return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
  //修改为固定的图像，而不是随机的
  //return spawnTurtle(name, x, y, angle, 11);

  // 检查名字是否以 "No" 开头并带数字
  size_t index = 11; // 默认图片索引
  if (name.size() > 2 && name.substr(0, 2) == "No") {
    try {
      int num = std::stoi(name.substr(2));
      // 假设图片数组下标和No编号一致，比如No2对应turtle_images_[2]
      if (num >= 0 && num < turtle_images_.size()) {
        index = num;
      }
    } catch (...) {
      // 解析失败，使用默认图片
    }
  }
  return spawnTurtle(name, x, y, angle, index);


}

std::string TurtleFrame::spawnTurtle(
  const std::string & name, float x, float y, float angle,
  size_t index)
{
  RCLCPP_INFO(nh_->get_logger(), "spawnTurtle called for [%s]", name.c_str());
  std::string real_name = name;
  if (real_name.empty()) {
    do{
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  } else {
    if (hasTurtle(real_name)) {
      return "";
    }
  }


  int modbus_port = 1502 + index;
//自己改的 height_in_meters_ - y 修改为y
  TurtlePtr t = std::make_shared<Turtle>(
    nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(
      x,
      y), angle, modbus_port,true);
  turtles_[real_name] = t;


  // 用线程异步初始化 Modbus，避免阻塞服务回调
  // std::thread([t](){
  //     try {
  //         t->init_modbus();
  //     } catch (const std::exception& e) {
  //         RCLCPP_ERROR(t->getLogger(), "Modbus init failed: %s", e.what());
  //     }
  // }).detach();


  update();

  RCLCPP_INFO(
    nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
    real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok()) {
    close();
    return;
  }

  rclcpp::spin_some(nh_);

  updateTurtles();
}

void TurtleFrame::paintEvent(QPaintEvent * event)
{
  (void)event;  // NO LINT
  QPainter painter(this);

  // int r = DEFAULT_BG_R;
  // int g = DEFAULT_BG_G;
  // int b = DEFAULT_BG_B;
  // nh_->get_parameter("background_r", r);
  // nh_->get_parameter("background_g", g);
  // nh_->get_parameter("background_b", b);
  // QRgb background_color = qRgb(r, g, b);
  // painter.fillRect(0, 0, width(), height(), background_color);

  //修改去除路径显示
  //painter.drawImage(QPoint(0, 0), path_image_);

  // 1. 收集所有乌龟的坐标信息
  QString all_coords_text;
  for (const auto& kv : turtles_) {
    const auto& name = kv.first;
    const auto& turtle = kv.second;
    double x = turtle->getPos().x();
    double y = turtle->getPos().y();
    double theta_deg = turtle->getOrient() * 180.0 / M_PI;
    double tvel=turtle->goal_vel;
    all_coords_text += QString("[%1: (%2, %3, %4, %5)]\n")
      .arg(QString::fromStdString(name), -4) // 名字左对齐宽8
      .arg(x, 6, 'f', 2, QChar(' '))         // x宽6，小数2位，空格补齐
      .arg(y, 6, 'f', 2, QChar(' '))
      .arg(theta_deg, 6, 'f', 2, QChar(' '))
      .arg(tvel, 6, 'f', 2, QChar(' '));
  }


  // 2. 绘制在画布右上角（纵向排列）
  painter.setPen(Qt::black);
  QFont font("Monospace"); // 或 QFont("Monospace")
  font.setStyleHint(QFont::Monospace); // 强制等宽
  font.setPointSize(8);
  painter.setFont(font);

  QFontMetrics fm(font);
  int text_width = 0;
  for (const auto& line : all_coords_text.split('\n')) {
    text_width = std::max(text_width, fm.horizontalAdvance(line));
  }
  int text_height = fm.height() * all_coords_text.count('\n');
  int margin = 10;

  // 右上角起点
  QPointF right_top_pos(width() - text_width - margin, margin*0 + fm.ascent());
  painter.drawText(QRectF(right_top_pos, QSizeF(text_width, text_height)), 
                    Qt::AlignLeft | Qt::AlignTop, all_coords_text);


  //调用乌龟的绘制函数
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    it->second->paint(painter);
  }


    // 1. 状态栏内容
    QString status_text = QString("当前AGV数量 %1")
        .arg(turtles_.size());

    // 2. 状态栏高度
    int status_height = 24;
    margin = 8;

    // 3. 绘制底色
    QRect status_rect(0, height() - status_height, width(), status_height);
    painter.fillRect(status_rect, QColor(230, 230, 230)); // 浅灰色

    // // 4. 绘制状态栏文字
    // painter.setPen(Qt::black);
    // font = painter.font();
    // font.setPointSize(10);
    // painter.setFont(font);
    // painter.drawText(status_rect.adjusted(margin, 0, -margin, 0), Qt::AlignVCenter | Qt::AlignLeft, status_text);


  // 按钮参数
  int button_count = 8;
  int button_width = 60;
  int button_height = 20;
  int button_spacing = 8;
  int button_top = height() - status_height + 2;
  int button_left = margin;

  if (button_states_.size() != button_count) button_states_.resize(button_count);

  // 关键：绘制按钮前单独设置按钮字体
  QFont buttonFont("Monospace");
  buttonFont.setPointSize(10); // 你想要的按钮字体大小
  painter.setFont(buttonFont);

  QVector<QRect> button_rects;
  for (int i = 0; i < button_count; ++i) {
      QRect btn_rect(button_left + i * (button_width + button_spacing), button_top, button_width, button_height);
      button_rects.append(btn_rect);
      //painter.setBrush(QColor(200, 200, 200));
      painter.setBrush(button_states_[i] ? QColor(120, 220, 120) : QColor(200, 200, 200));
      painter.setPen(Qt::black);
      painter.drawRect(btn_rect);
      painter.drawText(btn_rect, Qt::AlignCenter, QString("No%1").arg(i));
  }
  // 保存按钮区域，供鼠标事件用
  button_rects_ = button_rects; // 你需要在 TurtleFrame 类中加 QVector<QRect> button_rects_ 作为成员变量



}

void TurtleFrame::updateTurtles()
{

  
  // 先处理所有乌龟的 Modbus 请求
  for (auto& kv : turtles_) {
    kv.second->process_modbus_requests();
  }

  if (last_turtle_update_.nanoseconds() == 0) {
    last_turtle_update_ = nh_->now();
    return;
  }


  if (path_image_.isNull()) {
    RCLCPP_ERROR(nh_->get_logger(), "path_image_ is null!");
}

  // 只在这里begin
  if (!path_painter_.isActive()) {
    path_painter_.begin(&path_image_);
  }


  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    modified |= it->second->update(
      0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
      height_in_meters_);
  }


    // 只在这里end
  if (path_painter_.isActive()) {
    path_painter_.end();
  }



  if (modified) {
    update();
  }

  ++frame_count_;



  for (int i = 0; i < button_states_.size(); ++i) {
      QString name = QString("No%1").arg(i);
      button_states_[i] = hasTurtle(name.toStdString());
  }


}


bool TurtleFrame::clearCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing my_turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting my_turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}  // namespace my_turtlesim
