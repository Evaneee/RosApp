#include <QLineEdit>
#include <ctime>
#include "my_turtlesim/my_turtle_frame.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "my_turtlesim_msgs/msg/spawn_request.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace my_turtlesim
{


TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(800, 600, QImage::Format_ARGB32)
  , path_painter_()
  , frame_count_(0)
  , id_counter_(0)
{
  local_id = get_local_id_from_ip(); 
  setWindowTitle("乌龟控制");


  setAutoFillBackground(true);
  {
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(180, 180, 180));
    setPalette(pal);
  }


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

  //绘制8个文本框，按钮不需要绘制，因为不是真的按钮，只是画出来的区域
  for (int i = 0; i < button_count; ++i) {
      QLineEdit* edit = new QLineEdit(this);
      edit->setText("5.0,5.0,0.0");
      edit->setAlignment(Qt::AlignCenter);
      edit->show();
      turtle_coord_edits_.append(edit);
  }

  setMinimumSize(600, 200);   // 设置最小尺寸，防止太小
  resize(952, 600);           // 设置默认初始尺寸

  loadConfigFile("turtle_config.txt");
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
  turtle_coord_edits_.clear();
  turtle_images_.clear();
  turtles_.clear();
}



void TurtleFrame::spawnTopicCallback(const my_turtlesim_msgs::msg::SpawnRequest::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(), "spawnTopicCallback start");
  std::string name = spawnTurtle(msg->name, msg->x, msg->y, msg->theta/180.0*PI);
  if (name.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", msg->name.c_str());
    return;
  }
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

  update();
  RCLCPP_INFO(
    nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
    real_name.c_str(), x, y, angle/PI*180.0);

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
      modified |=it->second->highlight_error_;
  }

  checkTurtleCollisions();

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
