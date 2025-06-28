#include "my_turtlesim/my_turtle.hpp"

namespace my_turtlesim
{

    int cnt;
void Turtle::agv_state_machine()
{
    //程序段19：AGV运行中
    //Agv_Start_P在 step2时候会变成true
    db50_agv_.Sys.Agv_Start_P=Agv_Start_detector.detect(db50_agv_.Sys.Agv_Start);
    db50_agv_.Sys.Agv_Running=  (db50_agv_.Sys.Agv_Start_P ||   //只在step2设定为Agv_Running
                                 db50_agv_.Sys.Agv_Running )    //锁存Agv_Running
                             && (!db50_agv_.Sys.Agv_Comp);      //到step100将断掉Agv_Running

    //程序段19：AGV运行结束，step100中会置位Agv_Comp
    //Agv_Run_Proc 在step0后为true，step100回来下一周期所有变量赋值后为false
    if(db50_agv_.Sys.Agv_Comp)
    {
        db50_agv_.Sys.Agv_Comp=false;
        db50_agv_.Sys.Agv_Run_Proc=false;
    }


    //程序16：AGV运行启动
    //Agv_Start 在进入step0之前根据条件保持一个周期true，进入step0以后都是false，直到执行到step100的下一個周期
    db50_agv_.Sys.Agv_Start =   (!db50_agv_.Sys.Agv_Run_Proc) &&
                                (db51_tcp_.Tx_Agv.General.nodenum > 0);


    if(db51_tcp_.Tx_Agv.General.nodenum > 0)
    {
        cnt++;
    }


    // 单步运行标志位，step0以后任何一个step回来，Agv_Rsy_Cycle=true
    if (db50_agv_.Sys.Agv_Rsy_Cycle) {
        db50_agv_.Sys.Agv_Rsy_Cycle = false;
    }

    // 第0步，初始化
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 0 &&
        !db50_agv_.Sys.Agv_Run_Proc &&
        !db50_agv_.Sys.Agv_Comp)
    {
        RCLCPP_INFO(nh_->get_logger(), "[%s] step0: 状态机初始化", real_name.c_str());
        db50_agv_.Status.Mov_Fwd = false;
        db50_agv_.Status.Mov_Bwd = false;
        db50_agv_.Status.Mov_decel_Bypass = false;
        db50_agv_.Status.Mov_Stop_Bypass = false;
        db50_agv_.Status.Rotate_Fwd = false;
        db50_agv_.Status.Rotate_Bwd = false;

        db50_agv_.Int_Node_No = 0;

        db50_agv_.Excute.Locate_Start = false;
        db50_agv_.Excute.Rotation_Start = false;
        db50_agv_.Excute.MoveL_H_Start = false;
        db50_agv_.Excute.MoveL_V_Start = false;

        db50_agv_.Int_Step = 5;
        db50_agv_.Sys.Agv_Rsy_Cycle = true;
        db50_agv_.Sys.Agv_Run_Proc = true;
    }

    // 第5步，判断车身状态是否和上次任务时的状态是否一致，暂时没用
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 5)
    {
        db50_agv_.Sys.Agv_Rsy_Cycle = true;
        db50_agv_.Int_Step = 10;
    }

    // 第10步，读取目标节点信息
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 10)
    {
        
        int idx = db50_agv_.Int_Node_No;//取出一个新的子步
        if (idx < 0) idx = 0;
        if (idx > 9) idx = 9; // 防止越界

        db50_agv_.Node_SP.nodeno = db51_tcp_.Tx_Agv.Seq[idx].nodeno;
        RCLCPP_INFO(nh_->get_logger(), "[%s] step10: 读取modbus发布的子节点%d信息: nodeno=%d", real_name.c_str(),idx, db50_agv_.Node_SP.nodeno);
        db50_agv_.Node_SP.nodetype_action_sp = db51_tcp_.Tx_Agv.Seq[idx].nodetype_action_sp;
        db50_agv_.Node_SP.action_l = db51_tcp_.Tx_Agv.Seq[idx].action_l;
        db50_agv_.Node_SP.action_h = db51_tcp_.Tx_Agv.Seq[idx].action_h;
        db50_agv_.Node_SP.X = db51_tcp_.Tx_Agv.Seq[idx].X;
        db50_agv_.Node_SP.Y = db51_tcp_.Tx_Agv.Seq[idx].Y;
        db50_agv_.Node_SP.Rz = db51_tcp_.Tx_Agv.Seq[idx].Rz;

        uint16_t word_node_type_action = static_cast<uint16_t>(db50_agv_.Node_SP.nodetype_action_sp);
        db50_agv_.Data.Byte_Node_Type = (word_node_type_action >> 8) & 0xFF;
        db50_agv_.Data.Byte_Node_Action = word_node_type_action & 0xFF;

        uint16_t word_action_l = static_cast<uint16_t>(db50_agv_.Node_SP.action_l);
        db50_agv_.Data.Byte_Move_Action = word_action_l & 0x0F;
        db50_agv_.Data.Byte_Move_Speed = (word_action_l >> 4) & 0x03;
        db50_agv_.Data.Byte_MSG_LSP = (word_action_l >> 6) & 0x03;
        db50_agv_.Data.Bool_Radar_Enble = (word_action_l >> 8) & 0x01;
        db50_agv_.Data.Byte_Offline_Time = (word_action_l >> 9) & 0x07;
        db50_agv_.Data.Bool_Rotaion_Dir = (word_action_l >> 12) & 0x01;
        db50_agv_.Data.Bool_MoveC_Dir = (word_action_l >> 13) & 0x01;

        // 方向
        switch (db50_agv_.Data.Byte_Move_Action) {
            case 1: case 5:
                db50_agv_.Status.Mov_Fwd = true;
                db50_agv_.Status.Mov_Bwd = false;
                db50_agv_.Status.Mov_Left = false;
                db50_agv_.Status.Mov_Right = false;
                break;
            case 2: case 6:
                db50_agv_.Status.Mov_Fwd = false;
                db50_agv_.Status.Mov_Bwd = true;
                db50_agv_.Status.Mov_Left = false;
                db50_agv_.Status.Mov_Right = false;
                break;
            case 3: case 7:
                db50_agv_.Status.Mov_Fwd = false;
                db50_agv_.Status.Mov_Bwd = false;
                db50_agv_.Status.Mov_Left = true;
                db50_agv_.Status.Mov_Right = false;
                break;
            case 4: case 8:
                db50_agv_.Status.Mov_Fwd = false;
                db50_agv_.Status.Mov_Bwd = false;
                db50_agv_.Status.Mov_Left = false;
                db50_agv_.Status.Mov_Right = true;
                break;
            case 9:
                db50_agv_.Status.Mov_Fwd = false;
                db50_agv_.Status.Mov_Bwd = false;
                db50_agv_.Status.Mov_Left = false;
                db50_agv_.Status.Mov_Right = false;
                break;
            default:
                db50_agv_.Error[1] = true;
                RCLCPP_ERROR(nh_->get_logger(), "[%s] step10: Error[1], 工作模式action_l bit[0:3]超范围[1,9]", real_name.c_str());
                break;
        }

        // 旋转
        switch (db50_agv_.Data.Byte_Move_Action) {
            case 1: case 2: case 3: case 4: case 9:
                db50_agv_.Status.Rotate_Fwd = false;
                db50_agv_.Status.Rotate_Bwd = false;
                break;
            case 5: case 6: case 7: case 8:
                if (db50_agv_.Data.Bool_Rotaion_Dir)
                {
                    db50_agv_.Status.Rotate_Fwd = true;
                    db50_agv_.Status.Rotate_Bwd = false;
                }
                else
                {
                    db50_agv_.Status.Rotate_Fwd = false;
                    db50_agv_.Status.Rotate_Bwd = true;
                }
                break;
            default:
                db50_agv_.Error[1] = true;
                RCLCPP_ERROR(nh_->get_logger(), "[%s] step10: Error[1], 工作模式action_l bit[0:3]超范围[1,9]", real_name.c_str());
                break;
        }

        // 速度设定
        switch (db50_agv_.Data.Byte_Move_Speed) {
            case 0:
                db50_agv_.Real_Speedset = db51_tcp_.Tx_Agv.General.speedset_l / 1000.0f;
                break;
            case 1:
                db50_agv_.Real_Speedset = db51_tcp_.Tx_Agv.General.speedset_m / 1000.0f;
                break;
            case 2:
                db50_agv_.Real_Speedset = db51_tcp_.Tx_Agv.General.speedset_h / 1000.0f;
                break;
            default:
                db50_agv_.Error[1] = true;
                RCLCPP_ERROR(nh_->get_logger(), "[%s] step10: Error[1], 速度模式action_l bit[4:5]超范围[0,2]", real_name.c_str());
                break;
        }
        if (db50_agv_.Real_Speedset == 0) {
            db50_agv_.Error[1] = true;
            RCLCPP_ERROR(nh_->get_logger(), "[%s] step10: Error[1], 运行速度被配置为0", real_name.c_str());
        }

        db50_agv_.Dint_Move_dist = 0;

        if (!db50_agv_.Error[1]) {
            db50_agv_.Int_Step = 15;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } else {
            db50_agv_.Int_Step = 100;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        }
    }

    // 第15步，判断当前点是否为精确定位点
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 15)
    {
        RCLCPP_INFO(nh_->get_logger(), "[%s] step15: 判断当前点是否为精确定位点", real_name.c_str());
        if (db50_agv_.Data.Byte_Node_Action == 0) 
        {
            db50_agv_.Excute.Locate_Start = false;
            db50_agv_.Int_Step = 20;//如果没有特殊动作直接进入step20
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } 
        else if (db50_agv_.Data.Byte_Node_Action != 0) 
        {
            // 这里应调用定位程序，伪代码如下
            if (!db50_agv_.Excute.Locate_Start ) {
                db50_agv_.Real_Tar_Speed = 0.05f;
                db50_agv_.Excute.Locate_Start = true;
            }
            // 判断是否执行完成
            if (db50_agv_.Excute.Locate_Start) {
                db50_agv_.Excute.Locate_Start = false;
                db50_agv_.Int_Step = 20;//定位行走完成进入step20
                db50_agv_.Sys.Agv_Rsy_Cycle = true;
            }
        } 
        else 
        {
            db50_agv_.Error[2] = true;
            db50_agv_.Int_Step = 100;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        }
    } 
    else 
    {
        db50_agv_.Excute.Locate_Start = false;
    }

    // 第20步，是否为特殊动作点
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 20)
    {
        RCLCPP_INFO(nh_->get_logger(), "[%s] step20: 判断是否为特殊动作点", real_name.c_str());
        if (db50_agv_.Data.Byte_Node_Action == 0) {
            db50_agv_.Int_Step = 25;//如果没有特殊动作直接进入25
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } else if (db50_agv_.Data.Byte_Node_Action != 0) {
            switch (db50_agv_.Data.Byte_Node_Action) {
                case 1: // 充电
                case 2: // 来料取货
                case 3:
                case 4:
                    // TODO: 具体动作
                    break;
                case 5:
                    db50_agv_.Int_Step = 25;
                    db50_agv_.Sys.Agv_Rsy_Cycle = true;
                    break;
                default:
                    db50_agv_.Error[3] = true;
                    db50_agv_.Int_Step = 100;
                    db50_agv_.Sys.Agv_Rsy_Cycle = true;
                    break;
            }
        }
    }

    // 第25步，判断是否需要在当前点位置进行旋转
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 25)
    {
        //RCLCPP_INFO(nh_->get_logger(), "step25: 判断是否需要在当前点位置进行旋转");//执行过程会重复打印
        if (!db50_agv_.Status.Rotate_Fwd && !db50_agv_.Status.Rotate_Bwd) 
        {
            db50_agv_.Excute.Rotation_Start = false;
            db50_agv_.Int_Step = 30;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } 
        else if ((db50_agv_.Status.Rotate_Fwd && !db50_agv_.Status.Rotate_Bwd) ||
                   (!db50_agv_.Status.Rotate_Fwd && db50_agv_.Status.Rotate_Bwd)) 
        {
            // // 原旋转程序
            // if (!db50_agv_.Excute.Rotation_Start /*&& !DB40_Motion.Locate.Error*/) {
            //     // DB40_Motion.Locate.Action_Sel = false;
            //     db50_agv_.Real_Tar_Speed = 0.05f;
            //     db50_agv_.Excute.Rotation_Start = true;
            // }
            // // 判断是否执行完成
            // if (db50_agv_.Excute.Rotation_Start /*&& DB40_Motion.Rotate.Done*/) {
            //     db50_agv_.Excute.Rotation_Start = false;
            //     db50_agv_.Int_Step = 30;
            //     db50_agv_.Sys.Agv_Rsy_Cycle = true;
            // }

            /*-------------------------------------------------------------------------------------------------------------------------------------*/
            if (!db50_agv_.Excute.Rotation_Start && !client_walk_absolute_goal_handle_) 
            {
                auto goal_msg = my_turtlesim_msgs::action::WalkAbsolute::Goal();
                goal_msg.x1_y2_rote3 = 3; // 3表示旋转
                goal_msg.x = db50_agv_.Node_SP.X;
                goal_msg.y = db50_agv_.Node_SP.Y;
                goal_msg.theta = db50_agv_.Node_SP.Rz;

                auto send_goal_options = rclcpp_action::Client<my_turtlesim_msgs::action::WalkAbsolute>::SendGoalOptions();
                send_goal_options.result_callback = [this](const auto & result) 
                {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
                    {
                        RCLCPP_INFO(nh_->get_logger(), "[%s] --step25: 状态机收到action执行结果: walk_absolute action 原地回转执行成功", real_name.c_str());
                        db50_agv_.Excute.Rotation_Start = false;
                        db50_agv_.Int_Step = 30;
                        db50_agv_.Sys.Agv_Rsy_Cycle = true;
                        client_walk_absolute_goal_handle_ = nullptr;
                    } 
                    else 
                    {
                        RCLCPP_WARN(nh_->get_logger(), "[%s] --step25: 状态机收到action执行结果: walk_absolute action 原地回转执行失败或者取消", real_name.c_str());
                        db50_agv_.Excute.Rotation_Start = false;
                        db50_agv_.Error[4] = true;
                        db50_agv_.Int_Step = 100;
                        db50_agv_.Sys.Agv_Rsy_Cycle = true;
                        client_walk_absolute_goal_handle_ = nullptr;
                    }
                };
                // 发送action，并保存client_walk_absolute_goal_handle_future_
                RCLCPP_INFO(nh_->get_logger(), "[%s] step25: 发布ros旋转action(x=%f,y=%f,a=%f)", real_name.c_str(),goal_msg.x,goal_msg.y,goal_msg.theta);
                client_walk_absolute_goal_handle_future_ = walk_absolute_action_client_->async_send_goal(goal_msg,send_goal_options);
                db50_agv_.Excute.Rotation_Start = true;
            }

            // 检查action是否已被接受并获得client_walk_absolute_goal_handle_,上面的if就有作用了
            if (db50_agv_.Excute.Rotation_Start && client_walk_absolute_goal_handle_future_.valid() && !server_walk_absolute_goal_handle_) 
            {
                auto status = client_walk_absolute_goal_handle_future_.wait_for(std::chrono::seconds(0));
                if (status == std::future_status::ready) 
                {
                    client_walk_absolute_goal_handle_ = client_walk_absolute_goal_handle_future_.get();
                }
            }
          /*-------------------------------------------------------------------------------------------------------------------------------------*/

        } 
        else
        {
            RCLCPP_ERROR(nh_->get_logger(), "[%s] step25 Error[4], 同时指定前后回转, 进入Step100", real_name.c_str());
            db50_agv_.Error[4] = true;
            db50_agv_.Int_Step = 100;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        }
    } 
    else 
    {
        db50_agv_.Excute.Rotation_Start = false;
    }

    // 第30步，判断是否需要在当前点执行移动命令
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 30)
    {
        //RCLCPP_INFO(nh_->get_logger(), "step30: 判断是否需要在当前点执行移动命令");//执行过程会重复打印
        if (!db50_agv_.Status.Mov_Fwd && !db50_agv_.Status.Mov_Bwd &&
            !db50_agv_.Status.Mov_Left && !db50_agv_.Status.Mov_Right) {
            db50_agv_.Excute.MoveL_H_Start = false;
            db50_agv_.Excute.MoveL_V_Start = false;
            db50_agv_.Int_Step = 35;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } 
        else if (db50_agv_.Status.Mov_Fwd || db50_agv_.Status.Mov_Bwd ||
                   db50_agv_.Status.Mov_Left || db50_agv_.Status.Mov_Right) 
        {

/*-------------------------------------------------------------------------------------------------------------------------------------*/

            // // 这里应调用直行程序，伪代码如下
            // if ((db50_agv_.Status.Mov_Fwd || db50_agv_.Status.Mov_Bwd) &&
            //     !db50_agv_.Status.Mov_Left && !db50_agv_.Status.Mov_Right) {
            //     if (!db50_agv_.Excute.MoveL_H_Start /*&& !DB40_Motion.Trace.Done && !DB40_Motion.Trace.Error*/) {
            //         db50_agv_.Excute.MoveL_H_Start = true;
            //         if (db50_agv_.Status.Mov_Fwd)
            //             db50_agv_.Real_Tar_Speed = db50_agv_.Real_Speedset;
            //         else
            //             db50_agv_.Real_Tar_Speed = -db50_agv_.Real_Speedset;
            //     }
            // }
            // if ((db50_agv_.Status.Mov_Left || db50_agv_.Status.Mov_Right) &&
            //     !db50_agv_.Status.Mov_Fwd && !db50_agv_.Status.Mov_Bwd) {
            //     if (!db50_agv_.Excute.MoveL_V_Start /*&& !DB40_Motion.Trace.Done && !DB40_Motion.Trace.Error*/) {
            //         db50_agv_.Excute.MoveL_V_Start = true;
            //         if (db50_agv_.Status.Mov_Left)
            //             db50_agv_.Real_Tar_Speed = -db50_agv_.Real_Speedset;
            //         else
            //             db50_agv_.Real_Tar_Speed = db50_agv_.Real_Speedset;
            //     }
            // }
            // // 判断是否执行完成
            // if ((db50_agv_.Excute.MoveL_H_Start || db50_agv_.Excute.MoveL_V_Start) /*&& DB40_Motion.Trace.Done*/) {
            //     db50_agv_.Excute.MoveL_H_Start = false;
            //     db50_agv_.Excute.MoveL_V_Start = false;
            //     db50_agv_.Int_Step = 35;
            //     db50_agv_.Sys.Agv_Rsy_Cycle = true;
            // }

            // 直行动作（横向/纵向）统一用 action client 调用
            if (((db50_agv_.Status.Mov_Fwd || db50_agv_.Status.Mov_Bwd) && !db50_agv_.Status.Mov_Left && !db50_agv_.Status.Mov_Right) ||
                ((db50_agv_.Status.Mov_Left || db50_agv_.Status.Mov_Right) && !db50_agv_.Status.Mov_Fwd && !db50_agv_.Status.Mov_Bwd))
            {
                if (!db50_agv_.Excute.MoveL_H_Start && !db50_agv_.Excute.MoveL_V_Start && !client_walk_absolute_goal_handle_) 
                {
                    auto goal_msg = my_turtlesim_msgs::action::WalkAbsolute::Goal();

                    if(db50_agv_.Status.Mov_Fwd || db50_agv_.Status.Mov_Bwd)
                    {
                        goal_msg.x1_y2_rote3 = 2; // 2表示直行
                    }
                    else
                    {
                        goal_msg.x1_y2_rote3 = 1;
                    }

                    
                    goal_msg.x = db50_agv_.Node_SP.X;
                    goal_msg.y = db50_agv_.Node_SP.Y;
                    goal_msg.theta = db50_agv_.Node_SP.Rz;
                    goal_msg.vel=db50_agv_.Real_Speedset;

                    auto send_goal_options = rclcpp_action::Client<my_turtlesim_msgs::action::WalkAbsolute>::SendGoalOptions();
                    send_goal_options.result_callback = [this](const auto & result) 
                    {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
                        {
                            RCLCPP_INFO(nh_->get_logger(), "[%s] --step30: 状态机收到action执行结果: walk_absolute action 绝对值定位成功", real_name.c_str());
                            db50_agv_.Excute.MoveL_H_Start = false;
                            db50_agv_.Excute.MoveL_V_Start = false;
                            db50_agv_.Int_Step = 35;
                            db50_agv_.Sys.Agv_Rsy_Cycle = true;
                            client_walk_absolute_goal_handle_ = nullptr;
                        } 
                        else 
                        {
                            RCLCPP_WARN(nh_->get_logger(), "[%s] --step30: 状态机收到action执行结果: wwalk_absolute action 绝对值定位失败或者取消", real_name.c_str());
                            db50_agv_.Excute.MoveL_H_Start = false;
                            db50_agv_.Excute.MoveL_V_Start = false;
                            db50_agv_.Error[5] = true;
                            db50_agv_.Int_Step = 100;
                            db50_agv_.Sys.Agv_Rsy_Cycle = true;
                            client_walk_absolute_goal_handle_ = nullptr;
                        }
                    };
                    RCLCPP_INFO(nh_->get_logger(), "[%s] step30: 发布ros绝对值定位action(x=%0.2f,y=%0.2f,a=%0.2f,v=%0.2f,p=%d)", real_name.c_str(),goal_msg.x,goal_msg.y,goal_msg.theta,goal_msg.vel, goal_msg.x1_y2_rote3 );
                    client_walk_absolute_goal_handle_future_ = walk_absolute_action_client_->async_send_goal(goal_msg, send_goal_options);

                    // 标记开始直行
                    if (db50_agv_.Status.Mov_Fwd || db50_agv_.Status.Mov_Bwd)
                        db50_agv_.Excute.MoveL_H_Start = true;
                    if (db50_agv_.Status.Mov_Left || db50_agv_.Status.Mov_Right)
                        db50_agv_.Excute.MoveL_V_Start = true;
                }

                // 检查action是否已被接受并获得client_walk_absolute_goal_handle_
                if ((db50_agv_.Excute.MoveL_H_Start || db50_agv_.Excute.MoveL_V_Start) &&
                    client_walk_absolute_goal_handle_future_.valid() && !client_walk_absolute_goal_handle_) 
                {
                    auto status = client_walk_absolute_goal_handle_future_.wait_for(std::chrono::seconds(0));
                    if (status == std::future_status::ready) 
                    {
                        client_walk_absolute_goal_handle_ = client_walk_absolute_goal_handle_future_.get();
                    }
                }
            }
/*---------------------------------------------------------------------------------------------------------------*/
        } 
        else 
        {
            RCLCPP_ERROR(nh_->get_logger(), "[%s] step30: Error[5], 运行方向配置非预期", real_name.c_str());
            db50_agv_.Error[5] = true;
            db50_agv_.Int_Step = 100;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        }
    } else {
        db50_agv_.Excute.MoveL_H_Start = false;
        db50_agv_.Excute.MoveL_V_Start = false;
    }

    // 第35步，判断是否直行下一个节点
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 35)
    {
        
        if (db50_agv_.Int_Node_No < db51_tcp_.Tx_Agv.General.nodenum-1) {//这里和plc不一样，因为角标从0开始
            RCLCPP_INFO(nh_->get_logger(), "[%s] step35: 本节点任务完成, 节点号累加, 返回step10", real_name.c_str());
            db50_agv_.Int_Node_No += 1;
            db50_agv_.Int_Step = 10;//回到step 10，进行循环
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        } else {
            RCLCPP_INFO(nh_->get_logger(), "[%s] step35: 已结束所有节点任务, 进入step100", real_name.c_str());
            db50_agv_.Int_Step = 100;
            db50_agv_.Sys.Agv_Rsy_Cycle = true;
        }
    }

    // 第100步，结束，将数据保存
    if (db50_agv_.Sys.Agv_Running &&
        !db50_agv_.Sys.Agv_Rsy_Cycle &&
        db50_agv_.Int_Step == 100)
    {
        RCLCPP_INFO(nh_->get_logger(), "[%s] step100: 状态机结束", real_name.c_str());
        for (int i = 0; i < 16; ++i) {
            if (!db50_agv_.Sys.Agv_Alarm && db50_agv_.Error[i]) {
                db50_agv_.Sys.Agv_Alarm = true;
            }
        }
        if (!db50_agv_.Sys.Agv_Alarm) {
            db50_agv_.Node_Pre.nodeno = db50_agv_.Node_SP.nodeno;
            db50_agv_.Node_Pre.nodetype_action_sp = db50_agv_.Node_SP.nodetype_action_sp;
            db50_agv_.Node_Pre.action_l = db50_agv_.Node_SP.action_l;
            db50_agv_.Node_Pre.action_h = db50_agv_.Node_SP.action_h;
            db50_agv_.Node_Pre.X = db50_agv_.Node_SP.X;
            db50_agv_.Node_Pre.Y = db50_agv_.Node_SP.Y;
            db50_agv_.Node_Pre.Rz = db50_agv_.Node_SP.Rz;

            db50_agv_.Status.Wheels_0_deg_Pre = db50_agv_.Status.Wheels_0_deg;
            db50_agv_.Status.Wheels_p90_deg_Pre = db50_agv_.Status.Wheels_p90_deg;
            db50_agv_.Status.Wheels_m90_deg_Pre = db50_agv_.Status.Wheels_m90_deg;
        }
        db50_agv_.Sys.Agv_Comp = true;//跑完状态机的标识，无法再次进入step0，但是开头有自复位
        db50_agv_.Int_Step = 0;
        db50_agv_.Sys.Agv_Rsy_Cycle = true;
    }
}



}