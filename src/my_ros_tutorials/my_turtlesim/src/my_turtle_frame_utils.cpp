#include <QLineEdit>
#include <string>
#include <QPalette>    

#include <QFileDialog>
#include <QFile>
#include <QTextStream>

#include "my_turtlesim/my_turtle_frame.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "my_turtlesim_msgs/msg/spawn_request.hpp"

namespace my_turtlesim {

void TurtleFrame::loadConfigFile(const QString& filename)
{
    QFile file(filename);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        int idx = 0;
        while (!in.atEnd() && idx < turtle_coord_edits_.size()) {
            QString line = in.readLine().trimmed();
            // 校验格式：必须是“x,y,theta”三段数字
            QStringList parts = line.split(',');
            bool ok = false;
            if (parts.size() == 3) {
                bool ok1, ok2, ok3;
                parts[0].trimmed().toDouble(&ok1);
                parts[1].trimmed().toDouble(&ok2);
                parts[2].trimmed().toDouble(&ok3);
                ok = ok1 && ok2 && ok3;
            }
            turtle_coord_edits_[idx]->setText(line);
            if (ok) {
                turtle_coord_edits_[idx]->setPalette(QPalette()); // 恢复正常底色
            } else {
                QPalette pal = turtle_coord_edits_[idx]->palette();
                pal.setColor(QPalette::Base, QColor(255, 150, 150));
                turtle_coord_edits_[idx]->setPalette(pal);
            }
            ++idx;
        }
        file.close();
    }
}

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
  // 检查是否点击了生成按钮
  for (int i = 0; i < button_rects_.size(); ++i) {
      if (button_rects_[i].contains(event->pos())) {
          QString name = QString("No%1").arg(i);
          if (!button_states_[i]) {
            
            if (i < turtle_coord_edits_.size()) {
              QLineEdit* edit = turtle_coord_edits_[i];
              QString text = edit->text().trimmed();
              double x, y, theta;
              bool ok = false;
              QStringList parts = text.split(',');
              if (parts.size() == 3) {
                  bool ok1, ok2, ok3;
                  x = parts[0].trimmed().toDouble(&ok1);
                  y = parts[1].trimmed().toDouble(&ok2);
                  theta = parts[2].trimmed().toDouble(&ok3);
                  ok = ok1 && ok2 && ok3;
              }
              if (!ok) {
                  // 格式错误，文本框变红
                  QPalette pal = edit->palette();
                  pal.setColor(QPalette::Base, QColor(255, 150, 150));
                  edit->setPalette(pal);
                  return; // 不执行启动
              } else {
                  // 恢复正常颜色
                  edit->setPalette(QPalette());
              }

              auto pub = nh_->create_publisher<my_turtlesim_msgs::msg::SpawnRequest>("/spawn"+local_id, 10);
              my_turtlesim_msgs::msg::SpawnRequest msg;
              msg.x = x;
              msg.y = y;
              msg.theta = theta;
              msg.name = name.toStdString();
              pub->publish(msg);
            }

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
  }//按钮处理结束


  // 判断是否点中某只乌龟
  for (auto &kv : turtles_) {
      auto &turtle = kv.second;
      QPoint turtle_pos = QPoint(turtle->getPos().x() * meter_, turtle->getPos().y() * meter_);
      double radius = 15 * meter_; // 以乌龟半径为点击范围
      double delta=(event->pos() - turtle_pos).manhattanLength() ;
      if (delta< radius) {
          dragging_ = true;
          dragging_turtle_name_ = kv.first;
          drag_offset_ = event->pos() - turtle_pos;
          setCursor(Qt::ClosedHandCursor);
          return;
      }
  }//判断是否点中某只乌龟

  // 检查是否点击了保存或加载按钮
  if (save_button_rect_.contains(event->pos())) {
    QString filename = QFileDialog::getSaveFileName(this, "保存配置", "turtle_config.txt", "Text Files (*.txt)");
    if (!filename.isEmpty()) {
        QFile file(filename);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            for (QLineEdit* edit : turtle_coord_edits_) {
                out << edit->text() << "\n";
            }
            file.close();
        }
    }
    return;
  }// 检查是否点击了保存或加载按钮


  // 检查是否点击了加载按钮
  if (load_button_rect_.contains(event->pos())) {
    QString filename = QFileDialog::getOpenFileName(this, "加载配置", "", "Text Files (*.txt)");
    if (!filename.isEmpty()) {
        loadConfigFile(filename);
    }
    return;
  }// 检查是否点击了加载按钮

  QFrame::mousePressEvent(event);
}

void TurtleFrame::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging_ && turtles_.count(dragging_turtle_name_)) {
        auto &turtle = turtles_[dragging_turtle_name_];
        QPointF new_pos = (event->pos() - drag_offset_) / meter_;
        turtle->setPos(new_pos.x(), new_pos.y());
        update();
    }
    QFrame::mouseMoveEvent(event);
}

void TurtleFrame::mouseReleaseEvent(QMouseEvent *event)
{
    if (dragging_) {
        dragging_ = false;
        dragging_turtle_name_.clear();
        setCursor(Qt::ArrowCursor);
        update();
    }
    QFrame::mouseReleaseEvent(event);
}

void TurtleFrame::mouseDoubleClickEvent(QMouseEvent *event)
{
    // 判断是否点中某只乌龟
    for (auto &kv : turtles_) {
        auto &turtle = kv.second;
        QPoint turtle_pos = QPoint(turtle->getPos().x() * meter_, turtle->getPos().y() * meter_);
        double radius = 15 * meter_;
        double delta = (event->pos() - turtle_pos).manhattanLength();
        if (delta < radius) {
            double angle = turtle->getOrient();
            // 判断是否按下Ctrl
            if (event->modifiers() & Qt::ControlModifier) {
                angle -= 15.0 * M_PI / 180.0; // 逆向旋转10度
            } else {
                angle += 15.0 * M_PI / 180.0; // 正向旋转10度
            }
            turtle->setOrient(angle);
            turtle->highlight_error_ = false; // 取消高亮错误状态
            update();
            break;
        }
    }
    QFrame::mouseDoubleClickEvent(event);
}



void TurtleFrame::paintEvent(QPaintEvent * event)
{
    (void)event;  // NO LINT

    int margin = 8;
    int button_count = 8;
    int button_width = 100;
    int button_height = 20;
    int button_spacing = 4;
    int spacing = 4;
    int edit_height = 20;
    int status_height = button_height + edit_height + spacing + 8; // 8为上下边距
    int button_top = height() - status_height + 2;
    int button_left = margin;


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



    painter.setPen(Qt::black);
    QFont font("Monospace"); 
    font.setStyleHint(QFont::Monospace); 
    font.setPointSize(8);
    painter.setFont(font);

    QFontMetrics fm(font);
    int text_width = 0;
    for (const auto& line : all_coords_text.split('\n')) {
        text_width = std::max(text_width, fm.horizontalAdvance(line));
    }
    int text_height = fm.height() * all_coords_text.count('\n');
    

    // 画布右上角绘制坐标
    QPointF right_top_pos(width() - text_width - margin, margin*0 + fm.ascent());
    painter.drawText(QRectF(right_top_pos, QSizeF(text_width, text_height)), 
                    Qt::AlignLeft | Qt::AlignTop, all_coords_text);


    //调用乌龟的绘制函数
    M_Turtle::iterator it = turtles_.begin();
    M_Turtle::iterator end = turtles_.end();
    for (; it != end; ++it) {
        it->second->paint(painter);
    }


    // 绘制状态栏
    QRect status_rect(0, height() - status_height, width(), status_height);
    painter.fillRect(status_rect, QColor(230, 230, 230)); // 浅灰色


    if (button_states_.size() != button_count) button_states_.resize(button_count);

    QFont buttonFont("Monospace");
    buttonFont.setPointSize(10); // 你想要的按钮字体大小
    painter.setFont(buttonFont);

    //绘制按钮
    QVector<QRect> button_rects;
    for (int i = 0; i < button_count; ++i) {
        QRect btn_rect(button_left + i * (button_width + button_spacing), button_top, button_width, button_height);
        button_rects.append(btn_rect);
        painter.setBrush(button_states_[i] ? QColor(120, 220, 120) : QColor(200, 200, 200));
        painter.setPen(Qt::black);
        painter.drawRect(btn_rect);
        painter.drawText(btn_rect, Qt::AlignCenter, QString("No%1").arg(i));

        if (i < turtle_coord_edits_.size()) {
            QLineEdit* edit = turtle_coord_edits_[i];
            if (button_states_[i]) {
                edit->setDisabled(true); // 启动后禁用
            } else {
                edit->setDisabled(false); // 关闭后可编辑
            }
        }
    }
    button_rects_ = button_rects;

    //绘制文本框
    for (int i = 0; i < button_count && i < turtle_coord_edits_.size(); ++i) {
        QRect btn_rect = button_rects[i];
        QLineEdit* edit = turtle_coord_edits_[i];
        int edit_top = btn_rect.bottom() + spacing;
        edit->setGeometry(btn_rect.left(), edit_top, button_width, edit_height);
    }

    //绘制保存按钮
    QRect save_btn_rect(button_left + button_count * (button_width + button_spacing), button_top, button_width, button_height);
    painter.setBrush(QColor(180, 180, 255));
    painter.setPen(Qt::black);
    painter.drawRect(save_btn_rect);
    painter.drawText(save_btn_rect, Qt::AlignCenter, "保存配置");
    save_button_rect_ = save_btn_rect; 

    //绘制加载按钮
    QRect load_btn_rect(
    button_left + button_count * (button_width + button_spacing),
    button_top + button_height + spacing,
    button_width, button_height);
    painter.setBrush(QColor(180, 255, 180));
    painter.setPen(Qt::black);
    painter.drawRect(load_btn_rect);
    painter.drawText(load_btn_rect, Qt::AlignCenter, "加载配置");
    load_button_rect_ = load_btn_rect; 
}




}
