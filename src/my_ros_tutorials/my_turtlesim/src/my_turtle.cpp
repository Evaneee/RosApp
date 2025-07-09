

#include <sstream>
#include <iomanip>
#include "my_turtlesim/my_turtle.hpp"
#include <fcntl.h> // 文件顶部已包含
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <QColor>
#include <QRgb>

#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "my_turtlesim_msgs/action/rotate_absolute.hpp"
#include "my_turtlesim_msgs/action/walk_absolute.hpp"
#include "my_turtlesim_msgs/msg/pose.hpp"
#include "my_turtlesim_msgs/msg/color.hpp"
#include "my_turtlesim_msgs/srv/set_pen.hpp"
#include "my_turtlesim_msgs/srv/teleport_absolute.hpp"
#include "my_turtlesim_msgs/srv/teleport_relative.hpp"
#include "my_turtlesim/qos.hpp"

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace my_turtlesim
{


  QPointF alignPixel(const QPointF& p) {
    return QPointF(std::round(p.x()) + 0.5, std::round(p.y()) + 0.5);
}
// 使用时 painter.drawLine(alignPixel(a), alignPixel(b));

//被frame调用，，void TurtleFrame::paintEvent(QPaintEvent * event)
void Turtle::paint(QPainter & painter)
{

    // 1. 绘制目标点蓝色圆圈（假设有目标点）
  QPen pen(Qt::black);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.setBrush(QColor(255, 140, 0, 255)); // 橙色
  QPointF goal_pos = QPointF(goal_x_, goal_y_) * meter_;
  double arrow_len = 15; // 箭头长度（像素）
  double theta_rad = goal_theta_ * M_PI / 180.0;

  // 箭头终点
  QPointF arrow_tip = goal_pos + QPointF(std::cos(theta_rad), std::sin(theta_rad)) * arrow_len;

  // 主箭身
  painter.drawLine(goal_pos, arrow_tip);

  // 箭头两翼
  double wing_angle = 25.0 * M_PI / 180.0; // 翼与主箭身夹角25°
  double wing_len = 6; // 翼长

  QPointF left_wing = arrow_tip - QPointF(std::cos(theta_rad - wing_angle), std::sin(theta_rad - wing_angle)) * wing_len;
  QPointF right_wing = arrow_tip - QPointF(std::cos(theta_rad + wing_angle), std::sin(theta_rad + wing_angle)) * wing_len;

  painter.drawLine(arrow_tip, left_wing);
  painter.drawLine(arrow_tip, right_wing);

 
  // 可选：在箭头根部画个小圆点
  painter.setBrush(QColor(255, 140, 0, 180));
  painter.drawEllipse(goal_pos, 3, 3);



  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();

  bool draw_highlight = highlight_error_ && ((error_timer_.elapsed() / 250) % 2 == 0);
  if (draw_highlight) {
      painter.save();
      painter.setOpacity(0.5);
      painter.setBrush(Qt::red);
      painter.setPen(Qt::NoPen);
      painter.drawEllipse(p + QPointF(turtle_rotated_image_.width()/2, turtle_rotated_image_.height()/2), 24, 24);
      painter.restore();
  }

  painter.drawImage(p, turtle_rotated_image_);


    // 绘制当前位置坐标
  painter.setPen(Qt::black);
  //QFont font = painter.font();
  QFont font("Monospace");
  font.setPointSize(8);
  painter.setFont(font);
  // 坐标显示在乌龟图像的右上角
  double angle_deg = orient_ * 180.0 / M_PI;
  QString coord_text = QString("(%1, %2, %3, %4)")
                        .arg(pos_.x(), 0, 'f', 2)
                        .arg(pos_.y(), 0, 'f', 2)
                        .arg(angle_deg, 0, 'f', 2)
                        .arg(goal_vel, 0, 'f', 2);
  QPointF text_pos = p + QPointF(turtle_rotated_image_.width() / 2 + 8, -8);
  painter.drawText(text_pos, coord_text);
}


static double normalizeAngle(double angle)
{
  return angle - (TWO_PI * std::floor((angle + PI) / (TWO_PI)));
}

void Turtle::init_modbus()
{
// Modbus初始化
    modbus_ctx_ = modbus_new_tcp("0.0.0.0", modbus_port_);
    if (!modbus_ctx_) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to create Modbus context");
        throw std::runtime_error("Failed to create Modbus context");
    }

    modbus_mapping_ = modbus_mapping_new(150, 150, 150, 150);
    if (!modbus_mapping_) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to allocate modbus mapping");
        modbus_free(modbus_ctx_);
        throw std::runtime_error("Failed to allocate modbus mapping");
    }

    modbus_server_socket_fd_ = modbus_tcp_listen(modbus_ctx_, 1);
    if (modbus_server_socket_fd_ < 0) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to listen on Modbus port %d", modbus_port_);
        modbus_mapping_free(modbus_mapping_);
        modbus_free(modbus_ctx_);
        throw std::runtime_error("Failed to listen");
    }
    modbus_client_socket_fd_ = -1; // 初始为无连接



    // 打印Modbus信息
    RCLCPP_INFO(
      nh_->get_logger(),
      "Turtle [%s] Modbus slave(Tcp server) 启动, Tcp Port: %d, Modbus从站id: %d",
      real_name.c_str(), modbus_port_, modbus_port_ - 1501);
}


Turtle::Turtle(
  rclcpp::Node::SharedPtr & nh, const std::string & real_name,
  const QImage & turtle_image, const QPointF & pos, float orient, int modbus_port, bool delay_modbus)
: nh_(nh)
  , real_name(real_name) // ★★★ 这里必须加上
  , turtle_image_(turtle_image)
  , pos_(pos)
  , orient_(orient)
  , modbus_port_(modbus_port) // 添加modbus端口
  , lin_vel_x_(0.0)
  , lin_vel_y_(0.0)
  , ang_vel_(0.0)
  , pen_on_(true)
  , pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
{

  std::string local_id =  get_local_id_from_ip(); 



  RCLCPP_INFO(nh_->get_logger(), "Turtle [%s] constructor start at (%s)", real_name.c_str(),local_id.c_str());

  pen_.setWidth(3);

  const rclcpp::QoS qos = topic_qos();
  velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
    real_name + "/cmd_vel"+local_id, qos, std::bind(
      &Turtle::velocityCallback, this,
      std::placeholders::_1));

      
  pose_pub_ = nh_->create_publisher<my_turtlesim_msgs::msg::Pose>(real_name + "/pose"+local_id, qos);
  color_pub_ = nh_->create_publisher<my_turtlesim_msgs::msg::Color>(real_name + "/color_sensor"+local_id, qos);
  set_pen_srv_ =
    nh_->create_service<my_turtlesim_msgs::srv::SetPen>(
    real_name + "/set_pen"+local_id,
    std::bind(&Turtle::setPenCallback, this, std::placeholders::_1, std::placeholders::_2));
  teleport_relative_srv_ = nh_->create_service<my_turtlesim_msgs::srv::TeleportRelative>(
    real_name + "/teleport_relative"+local_id,
    std::bind(
      &Turtle::teleportRelativeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  teleport_absolute_srv_ = nh_->create_service<my_turtlesim_msgs::srv::TeleportAbsolute>(
    real_name + "/teleport_absolute"+local_id,
    std::bind(
      &Turtle::teleportAbsoluteCallback, this, std::placeholders::_1,
      std::placeholders::_2));



  rotate_absolute_action_server_ =
    rclcpp_action::create_server<my_turtlesim_msgs::action::RotateAbsolute>(
      nh,
      real_name + "/rotate_absolute"+local_id,
    [](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const my_turtlesim_msgs::action::RotateAbsolute::Goal>)
    {
        // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [](const std::shared_ptr<RotateAbsoluteGoalHandle>)
    {
        // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&Turtle::rotateAbsoluteAcceptCallback, this, std::placeholders::_1));


    //自己加的server-开始
    walk_absolute_action_server_ =
    rclcpp_action::create_server<my_turtlesim_msgs::action::WalkAbsolute>(
      nh, real_name + "/walk_absolute"+local_id,
    [](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const my_turtlesim_msgs::action::WalkAbsolute::Goal>)
    {
        // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<WalkAbsoluteGoalHandle>)
    {
        // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&Turtle::walkAbsoluteAcceptCallback, this, std::placeholders::_1));
    //自己加的server-结束

    //自己加的client开始
    walk_absolute_action_client_ = 
    rclcpp_action::create_client<my_turtlesim_msgs::action::WalkAbsolute>(
        nh_, real_name + "/walk_absolute"+local_id);
    //自己加的client结束




    last_command_time_ = nh_->now();
    meter_ = turtle_image_.height()/30.0;
    rotateImage();

    (void)delay_modbus;
    // if (!delay_modbus) {
    //     init_modbus();
    // }

    init_modbus();
    
    if (meter_ == 0) {
        RCLCPP_ERROR(nh_->get_logger(), "meter_ is 0! Image height is 0.");
        throw std::runtime_error("meter_ is 0");
    }

    error_timer_.start();
    RCLCPP_INFO(nh_->get_logger(), "Turtle [%s] 构造完成, action servers处于ready状态", real_name.c_str());

}


void Turtle::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
  last_command_time_ = nh_->now();
  lin_vel_x_ = vel->linear.x;
  bool holonomic = false;
  nh_->get_parameter_or("holonomic", holonomic, false);
  if (holonomic) {
    lin_vel_y_ = vel->linear.y;
  }
  ang_vel_ = vel->angular.z;

  // Abort any active action
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(nh_->get_logger(), "Velocity command received during rotation goal. Aborting goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
    rotate_absolute_goal_handle_ = nullptr;
  }
}

bool Turtle::setPenCallback(
  const my_turtlesim_msgs::srv::SetPen::Request::SharedPtr req,
  my_turtlesim_msgs::srv::SetPen::Response::SharedPtr)
{
  pen_on_ = !req->off;
  if (req->off) {
    return true;
  }

  QPen pen(QColor(req->r, req->g, req->b));
  if (req->width != 0) {
    pen.setWidth(req->width);
  }

  pen_ = pen;
  return true;
}

bool Turtle::teleportRelativeCallback(
  const my_turtlesim_msgs::srv::TeleportRelative::Request::SharedPtr req,
  my_turtlesim_msgs::srv::TeleportRelative::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(0, 0, req->angular, req->linear, true));
  return true;
}

bool Turtle::teleportAbsoluteCallback(
  const my_turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr req,
  my_turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(req->x, req->y, req->theta, 0, false));
  return true;
}
//旋转一层回调函数
void Turtle::rotateAbsoluteAcceptCallback(
  const std::shared_ptr<RotateAbsoluteGoalHandle> goal_handle)
{
  // Abort any existing goal
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "Rotation goal received before a previous goal finished. Aborting previous goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
  }
  rotate_absolute_goal_handle_ = goal_handle;
  rotate_absolute_feedback_.reset(new my_turtlesim_msgs::action::RotateAbsolute::Feedback);
  rotate_absolute_result_.reset(new my_turtlesim_msgs::action::RotateAbsolute::Result);
  rotate_absolute_start_orient_ = orient_;
}

//绝对值走位一层回调函数
void Turtle::walkAbsoluteAcceptCallback(
  const std::shared_ptr<WalkAbsoluteGoalHandle> goal_handle)
{
  server_walk_absolute_goal_handle_ = goal_handle;

  server_walk_absolute_feedback_.reset(new my_turtlesim_msgs::action::WalkAbsolute::Feedback);
  server_walk_absolute_result_.reset(new my_turtlesim_msgs::action::WalkAbsolute::Result);
  server_walk_absolute_start_orient_ = orient_;
}


void Turtle::rotateImage()
{
  QTransform transform;
  transform.rotate(+orient_ * 180.0 / PI + 90.0);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}


void Turtle::setOrient(double angle) {
        orient_ = angle;
        rotateImage(); // 旋转后刷新乌龟图像
    }



/*======================================================================================================================================================================= */

bool Turtle::update(
  double dt, QPainter & path_painter, const QImage & path_image,
  qreal canvas_width, qreal canvas_height)
{

  (void)canvas_width;
  (void)canvas_height;

  bool modified = false;
  qreal old_orient = orient_;
  syn_agv_para();

  //测试出结果，正常坐标系，点在直线高角度一侧，返回值为负
  // double tmp;
  // tmp=pointToLineSignedDistance(QPointF(300,200),500,500,30);
  // RCLCPP_INFO(nh_->get_logger(), "tmp = %f", tmp);
  // tmp=pointToLineSignedDistance(QPointF(200,500),500,500,30);
  // RCLCPP_INFO(nh_->get_logger(), "tmp = %f", tmp);



// first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it) {
    const TeleportRequest & req = *it;

    QPointF old_pos = pos_;
    if (req.relative) {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += -std::sin(orient_) * req.linear;
    } else {
      pos_.setX(req.pos.x());
      //自己改的，新生成的小乌龟使用画布坐标
      //pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      pos_.setY(req.pos.y());
      orient_ = req.theta;
    }

    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // Process any action requests
  if (rotate_absolute_goal_handle_) 
  {
    // Check if there was a cancel request
    if (rotate_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "Rotation goal canceled");
      rotate_absolute_goal_handle_->canceled(rotate_absolute_result_);
      rotate_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } 
    else 
    {
      double theta = normalizeAngle(rotate_absolute_goal_handle_->get_goal()->theta);
      double remaining = normalizeAngle(theta - static_cast<float>(orient_));

      // Update result
      rotate_absolute_result_->delta =
        normalizeAngle(static_cast<float>(rotate_absolute_start_orient_ - orient_));

      // Update feedback
      rotate_absolute_feedback_->remaining = remaining;
      rotate_absolute_goal_handle_->publish_feedback(rotate_absolute_feedback_);

      // Check stopping condition
      if (fabs(normalizeAngle(static_cast<float>(orient_) - theta)) < 0.02) {
        RCLCPP_INFO(nh_->get_logger(), "Rotation goal completed successfully");
        rotate_absolute_goal_handle_->succeed(rotate_absolute_result_);
        rotate_absolute_goal_handle_ = nullptr;
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      } else {
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = remaining < 0.0 ? -1.0 : 1.0;
        last_command_time_ = nh_->now();
      }
    }
  }//处理完rotate _absolute_goal_handle_

  //处理绝对值定位action
  if (server_walk_absolute_goal_handle_) 
  {
    if (server_walk_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "绝对值走位目标已取消");
      server_walk_absolute_goal_handle_->canceled(server_walk_absolute_result_);
      server_walk_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } else {


//麦轮形式的闭环
/*
      auto goal = server_walk_absolute_goal_handle_->get_goal();
      double dx = goal->x - pos_.x();
      double dy = goal->y - pos_.y();
      double dist = std::sqrt(dx * dx + dy * dy);
      double target_angle = std::atan2(-dy, dx); 
      double angle_to_target = normalizeAngle(target_angle - orient_);

      
      

      double v = (dist >= 1) ? goal->vel*20.0 : dist*goal->vel*20.0;
      goal_vel=goal->vel;

      lin_vel_x_ = v * std::cos(angle_to_target);
      lin_vel_y_ = v * std::sin(angle_to_target); 

      if (goal->walk0_rote1==1)
      {
        lin_vel_x_=0;
        lin_vel_y_=0;
      }

      ang_vel_ = 0.0;
      double ang_jacobi=10.0;
      double ang_vel_max=10.0;
      double ang_p=ang_vel_max/ang_jacobi;
      double goal_theta_rad = goal->theta * M_PI / 180.0;
      double theta_error = normalizeAngle(goal_theta_rad - orient_);
      if(std::abs(theta_error) > ang_jacobi)
      {
        ang_vel_ = (theta_error <= 0 ? -ang_vel_max: ang_vel_max);
      }
      else
      {
        ang_vel_ = theta_error *ang_p;
      }

*/
//////////////////////////////////////////////////////////////////////////////////////////

      int cnt2=0;
      double pos_x,pos_y,pos_theta;
      pos_x=pos_.x();
      pos_y=pos_.y();
      pos_theta=orient_/M_PI*180.0;
      if (false)
      {
        cnt2++;
        RCLCPP_INFO(nh_->get_logger(), "%f,%f,%f", pos_x,pos_y,pos_theta);
      }

      auto goal = server_walk_absolute_goal_handle_->get_goal();
      goal_x_ = goal->x;
      goal_y_ = goal->y;
      goal_theta_ = goal->theta;

      double dx = goal->x - pos_.x();
      double dy = goal->y - pos_.y();
      double dist = std::sqrt(dx * dx + dy * dy);
      double target_angle=0;
      if (dist>0.001)
        target_angle = std::atan2(dy, dx); 
      else
      {
        if(goal->x1_y2_rote3>=2)
        {
          target_angle = orient_; //如果距离小于0.1m，直接使用当前
        }
        else if(goal->x1_y2_rote3==1)
        {
          target_angle = orient_ + M_PI/2.0; //如果距离小于0.1m，直接使用当前
        }
          
      }
        

      double goal_theta_rad = goal->theta * M_PI / 180.0;
      double theta_error = normalizeAngle(goal_theta_rad - orient_);
      goal_vel=goal->vel;

      double angle_end_to_target ;
      double dist_project;
      double dist_project_limit;
      double v;
      double orient_deg;
      double end_to_orient;
      double df;
      double db;

      double v_is_pos;
      double ang_ft;
      double ang_bt;

      double angm_to_orient;

      if (goal->x1_y2_rote3==2)
      {
        angle_end_to_target = normalizeAngle(target_angle - goal_theta_rad);
        dist_project = dist * std::cos(angle_end_to_target); // 计算目标点到直线的投影距离
        dist_project_limit=mylimit(dist_project,-10,10); //大于0意味着正向正确
        v=goal->vel*4.0*dist_project_limit;

        orient_deg = orient_ / M_PI * 180.0;
        end_to_orient = orient_deg- goal->theta;

      
        df=pointToLineSignedDistance(point_f,goal->x,goal->y,goal->theta);
        db=pointToLineSignedDistance(point_b,goal->x,goal->y,goal->theta);

        if(abs(angle_end_to_target)<PI/3.0)
        {
          v_is_pos=1.0;
          highlight_error_ = false;
        }
        else if((PI-abs(angle_end_to_target))<PI/3.0)
        {
          v_is_pos=-1.0;
          highlight_error_ = false;
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "[%s] 纵向无法行驶到目标点, 目标角与指向角偏离量超容差des=%0.2f tar=%0.2f, 任务放弃", real_name.c_str(), goal_theta_rad/PI*180.0,target_angle/PI*180.0);
          server_walk_absolute_goal_handle_->abort(server_walk_absolute_result_);
          server_walk_absolute_goal_handle_ = nullptr;

          lin_vel_x_ = 0.0;
          lin_vel_y_ = 0.0;
          ang_vel_ = 0.0;

          highlight_error_ = true;
          error_timer_.restart();
          
          
        }
         ang_ft=-df*45.0*v_is_pos;  
         ang_bt=-db*45.0*v_is_pos;  

        ang_f=mylimit(ang_ft+end_to_orient,-45,45);//补充车体相对目的曲线转角
        ang_b=mylimit(ang_bt+end_to_orient,-45,45);
        
        calc_ang(ang_f,ang_b,HH,0,0,&ang_m,&now_R);

        angm_to_orient = (orient_deg- ang_m);
        lin_vel_x_=v* cos(angm_to_orient/180.0*PI);
        lin_vel_y_=v* sin(angm_to_orient/180.0*PI);
        ang_vel_=-v/now_R;
      }


      else if (goal->x1_y2_rote3==1)
      {
        angle_end_to_target = normalizeAngle(target_angle - (goal_theta_rad+PI/2.0));
        dist_project = dist * std::cos(angle_end_to_target); // 计算目标点到直线的投影距离
        dist_project_limit=mylimit(dist_project,-10,10); //大于0意味着正向正确
        v=-goal->vel*4.0*dist_project_limit;

        orient_deg = orient_ / M_PI * 180.0+90.0;
        end_to_orient = orient_deg- (goal->theta+90);

        df=pointToLineSignedDistance(point_r,goal->x,goal->y,goal->theta+90);
        db=pointToLineSignedDistance(point_l,goal->x,goal->y,goal->theta+90);

        if(abs(angle_end_to_target)<PI/3.0)
        {
          v_is_pos=1.0;
        }
        else if((PI-abs(angle_end_to_target))<PI/3.0)
        {
          v_is_pos=-1.0;
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "[%s] 横向无法行驶到目标点, 目标角与指向角偏离量超容差des=%0.2f tar=%0.2f, 任务放弃", real_name.c_str(), goal_theta_rad/PI*180.0,target_angle/PI*180.0);

          server_walk_absolute_goal_handle_->abort(server_walk_absolute_result_);
          server_walk_absolute_goal_handle_ = nullptr;

          lin_vel_x_ = 0.0;
          lin_vel_y_ = 0.0;
          ang_vel_ = 0.0;


          highlight_error_ = true;
          error_timer_.restart();


        }

        ang_ft=-df*45.0*v_is_pos;  
        ang_bt=-db*45.0*v_is_pos;  

        ang_f=mylimit(ang_ft+end_to_orient,-45,45);//补充车体相对目的曲线转角
        ang_b=mylimit(ang_bt+end_to_orient,-45,45);
        
        calc_ang(ang_f,ang_b,HH,0,0,&ang_m,&now_R);

        angm_to_orient = (orient_deg+90- ang_m);
        lin_vel_y_=v* cos(angm_to_orient/180.0*PI);
        lin_vel_x_=-v* sin(angm_to_orient/180.0*PI);
        ang_vel_=v/now_R;
      }

      else if (goal->x1_y2_rote3==3)
      {
        ang_vel_ = 0.0;
        double ang_jacobi=10.0;
        double ang_vel_max=10.0;

        double ang_p=ang_vel_max/ang_jacobi;

        if(std::abs(theta_error) > ang_jacobi)
        {
          ang_vel_ = (theta_error <= 0 ? -ang_vel_max: ang_vel_max);
        }
        else
        {
          ang_vel_ = theta_error *ang_p;
        }

        lin_vel_x_=0;
        lin_vel_y_=0;

      }




      // if(ang_vel_>0.2)
      // {
      //   cnt2++;
      // }
///////////////////////////////////////////////////////////////////////////////////////////




      // 发布反馈

      server_walk_absolute_feedback_->remaining = dist;
      //server_walk_absolute_goal_handle_->publish_feedback(server_walk_absolute_feedback_);

      // 三个条件都满足才算完成
      bool dist_ok = ((dist < 0.001)&& (goal->x1_y2_rote3==1 || goal->x1_y2_rote3==2)) || (goal->x1_y2_rote3==3);
      if (dist_ok && std::abs(theta_error) < 0.001) {
        RCLCPP_INFO(nh_->get_logger(), "[%s] ros action server消息: 绝对值action完成", real_name.c_str());
        server_walk_absolute_result_->delta = dist;
        if(server_walk_absolute_goal_handle_)
        {
          server_walk_absolute_goal_handle_->succeed(server_walk_absolute_result_);
          server_walk_absolute_goal_handle_ = nullptr;
        }
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      }
      last_command_time_ = nh_->now();
    }
  }//server_walk_absolute_goal_handle_处理结束



  if (nh_->now() - last_command_time_ > rclcpp::Duration(1.0, 0)) {
    lin_vel_x_ = 0.0;
    lin_vel_y_ = 0.0;
    ang_vel_ = 0.0;
  }


  //更新小乌龟坐标，更新坐标后，会在paint函数中绘制小乌龟
  QPointF old_pos = pos_;
  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ = normalizeAngle(orient_);
  //pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt - std::sin(orient_) * lin_vel_y_ * dt;
  //pos_.ry() -= std::cos(orient_) * lin_vel_y_ * dt + std::sin(orient_) * lin_vel_x_ * dt;

  //pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt + std::sin(orient_) * lin_vel_y_ * dt;
  //pos_.ry() += std::cos(orient_) * lin_vel_y_ * dt + std::sin(orient_) * lin_vel_x_ * dt;

  // pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt;
  // pos_.ry() += std::cos(orient_) * lin_vel_y_ * dt;

  pos_.rx() += lin_vel_x_ * dt;
  pos_.ry() += lin_vel_y_ * dt;



  db51_tcp_.Rx_Agv.heartbeat++;
  db51_tcp_.Rx_Agv.agvno   = modbus_port_ - 1502; // 写入乌龟编号
  db51_tcp_.Rx_Agv.station = db50_agv_.Node_SP.nodeno;

  db51_tcp_.Rx_Agv.nodetype_action_sp = db50_agv_.Node_SP.nodetype_action_sp;
  db51_tcp_.Rx_Agv.action_l = db50_agv_.Node_SP.action_l;
  db51_tcp_.Rx_Agv.action_h = db50_agv_.Node_SP.action_h;

  db51_tcp_.Rx_Agv.taskstate = db50_agv_.Int_Step;  // 任务执行状态(0执行完毕/1正在执行任务/ 2任务暂停 / 3任务错误终止)
  //db51_tcp_.Rx_Agv.runstate = db50_agv_.Int_Step;   //待确认
  //db51_tcp_.Rx_Agv.execute_state1;// 执行机构状态1(1:顶升中, 2:下降中,3:顶升到位 ,4:下降到位
  //db51_tcp_.Rx_Agv.execute_state2;// 若执行机构为顶升，此字段可为：升降高度数据     单位为mm
  db51_tcp_.Rx_Agv.speed=(int16_t)(goal_vel*100); // 速度设置，单位为cm/s
  db51_tcp_.Rx_Agv.electricity=97;

  int16_t result = 0;
  for (int i = 0; i < 16; ++i) {
      if (db50_agv_.Error[i]) {
          result |= (1 << i);
      }
  }
  db51_tcp_.Rx_Agv.alarm = result;//待确认,是否有车体错误

  db51_tcp_.Rx_Agv.laser_x = pos_.x();
  db51_tcp_.Rx_Agv.laser_y = pos_.y();
  db51_tcp_.Rx_Agv.laser_a = orient_/PI*180.0;
  db51_tcp_.Rx_Agv.targetstation=db51_tcp_.Tx_Agv.General.targetstation;
  db51_tcp_.Rx_Agv.tasktype=  db51_tcp_.Tx_Agv.General.tasktype;  // 任务类型，看起来没用到
  db51_tcp_.Rx_Agv.enable=db50_agv_.Sys.Agv_Running;
  db51_tcp_.Rx_Agv.speedset_l=db51_tcp_.Tx_Agv.General.speedset_l;
  db51_tcp_.Rx_Agv.speedset_m=db51_tcp_.Tx_Agv.General.speedset_m;
  db51_tcp_.Rx_Agv.speedset_h=db51_tcp_.Tx_Agv.General.speedset_h;
  db51_tcp_.Rx_Agv.nodenum=db51_tcp_.Tx_Agv.General.nodenum;
  
  Rx_struct_to_modbus(db51_tcp_.Rx_Agv, modbus_mapping_, 0); // base_addr按你的实际情况


  // Publish pose of the turtle
  auto p = std::make_unique<my_turtlesim_msgs::msg::Pose>();
  p->x = pos_.x();
  //自己改的，新生成的小乌龟使用画布坐标
  //p->y = canvas_height - pos_.y();
  p->y = pos_.y();//这是更改内容
  p->theta = orient_;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  // Figure out (and publish) the color underneath the turtle
  {
    auto color = std::make_unique<my_turtlesim_msgs::msg::Color>();

    QPoint pt = (pos_ * meter_).toPoint();
    if (pt.x() >= 0 && pt.x() < path_image.width() &&
        pt.y() >= 0 && pt.y() < path_image.height()) {
      QRgb pixel = path_image.pixel(pt);

      //QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
      color->r = qRed(pixel);
      color->g = qGreen(pixel);
      color->b = qBlue(pixel);
    } else {
      // 超出画布，给默认颜色（如白色）
      color->r = 255;
      color->g = 255;
      color->b = 255;
    }



    
    color_pub_->publish(std::move(color));
  }

  RCLCPP_DEBUG(
    nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f",
    nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient) {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos) {
    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}//update结束
/*======================================================================================================================================================================= */

Turtle::~Turtle()
{
    if (modbus_server_socket_fd_ >= 0) {
        ::close(modbus_server_socket_fd_);
        modbus_server_socket_fd_ = -1;
    }
    if (modbus_client_socket_fd_ >= 0) {
        ::close(modbus_client_socket_fd_);
        modbus_client_socket_fd_ = -1;
    }
    if (modbus_mapping_) {
        modbus_mapping_free(modbus_mapping_);
        modbus_mapping_ = nullptr;
    }
    if (modbus_ctx_) {
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
    }
    RCLCPP_INFO(nh_->get_logger(), "Turtle [%s] Modbus对象资源已释放", real_name.c_str());
}











}  // namespace my_turtlesim
