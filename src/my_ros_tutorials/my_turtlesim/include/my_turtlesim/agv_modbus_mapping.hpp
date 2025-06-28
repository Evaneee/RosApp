#ifndef TURTLESIM__AGV_MODBUS_MAPPING_HPP_
#define TURTLESIM__AGV_MODBUS_MAPPING_HPP_

#include <modbus/modbus.h>
#include <cstdint>
#include <string>

namespace my_turtlesim {

struct AGVGeneral {
    int16_t heartbeat;
    int16_t agvno;
    int16_t targetstation;
    int16_t tasktype;
    int16_t taskctrl;
    int16_t speedset_l;
    int16_t speedset_m;
    int16_t speedset_h;
    int16_t nodenum;
    int16_t agvctrl;
    int16_t reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, reserve9, reserve10;
};

struct AGVSeq {
    int16_t nodeno;
    int16_t nodetype_action_sp;
    int16_t action_l;
    int16_t action_h;
    float X;
    float Y;
    float Rz;
};

struct Tx_MODBUS_STRUCT {
    AGVGeneral General;
    AGVSeq Seq[10];
};


// ------------------- Rx_AGV 结构体 -------------------
struct Rx_MODBUS_STRUCT {
    int16_t heartbeat;
    int16_t agvno;
    int16_t station;
    int16_t reserve;
    int16_t nodetype_action_sp;
    int16_t action_l;
    int16_t action_h;
    int16_t taskstate;
    int16_t runstate;
    int16_t execute_state1;
    int16_t execute_state2;
    int16_t speed;
    int16_t electricity;
    int16_t alarm;
    float laser_x;
    float laser_y;
    float laser_a;
    int16_t targetstation;
    int16_t reserve1;
    int16_t tasktype;
    int16_t enable;
    int16_t speedset_l;
    int16_t speedset_m;
    int16_t speedset_h;
    int16_t nodenum;
    int16_t reserve_26;
    int16_t reserve_27;
};

struct DB51_TCP{
    Rx_MODBUS_STRUCT Rx_Agv;
    Tx_MODBUS_STRUCT Tx_Agv;
};



//////////////////////////////////////////////////////////////////////////
// DB50系统变量
struct DB50_Sys {
    bool Agv_Ready;
    bool Agv_Start;
    bool Agv_Running;
    bool Agv_Comp;
    bool Agv_Pause;
    bool Agv_Interrupt;
    bool Agv_Reset;
    bool Agv_Restart;
    bool Agv_Alarm;
    bool Agv_Rsy_Cycle;
    bool Agv_Run_Proc;
    bool Agv_Start_P;
    bool Agv_Stop;
    bool Res13, Res14, Res15;
    bool Agv_Inter_Area;
    bool Obst_F_Bypass, Obst_B_Bypass, Obst_R_Bypass, Obst_L_Bypass, Obst_All_Bypass;
};

// 状态变量
struct DB50_Status {
    bool Wheels_0_deg;
    bool Wheels_m90_deg;
    bool Wheels_p90_deg;
    bool Body_0_deg;
    bool Body_90_deg;
    bool Res5, Res6, Res7;
    bool Agv_Pos_ok;
    bool Agv_Track_ok;
    bool Agv_Track_ng;
    bool Agv_Obst_ok;
    bool Agv_Del_Pos;
    bool Agv_Node_Pos;
    bool Agv_Acc_Pos;
    bool Res15;
    bool Mov_Fwd;
    bool Mov_Bwd;
    bool Mov_decel_Bypass;
    bool Mov_Stop_Bypass;
    bool Mov_Speed_H;
    bool Mov_Left;
    bool Mov_Right;
    bool Res23;
    bool Rotate_Fwd;
    bool Rotate_Bwd;
    bool Res26, Res27, Res28, Res29, Res30, Res31;
    bool Heartbeat;
    bool Comm_Ok;
    bool Charge_Start;
    bool Steering_On;
    bool Res36, Res37, Res38, Res39;
    bool Wheels_0_deg_Pre;
    bool Wheels_m90_deg_Pre;
    bool Wheels_p90_deg_Pre;
    bool Body_0_deg_Pre;
    bool Body_90_deg_Pre;
    bool Res45, Res46, Res47;
    bool FL_Wheel_Fwd_LS;
    bool FL_Wheel_Bwd_LS;
    bool FR_Wheel_Fwd_LS;
    bool FR_Wheel_Bwd_LS;
    bool RL_Wheel_Fwd_LS;
    bool RL_Wheel_Bwd_LS;
    bool RR_Wheel_Fwd_LS;
    bool RR_Wheel_Bwd_LS;
    bool Res56, Res57, Res58, Res59, Res60, Res61, Res62, Res63;
};

// 执行动作
struct DB50_Excute {
    bool Locate_Start;
    bool Rotation_Start;
    bool MoveL_H_Start;
    bool MoveL_V_Start;
};

// 解析数据
struct DB50_Data {
    uint8_t Byte_Node_Type;
    uint8_t Byte_Node_Action;
    uint8_t Byte_Move_Action;
    uint8_t Byte_Move_Speed;
    uint8_t Byte_MSG_LSP;
    bool Bool_Radar_Enble;
    bool Bool_Rotaion_Dir;
    bool Bool_MoveC_Dir;
    uint8_t Byte_Offline_Time;
};

// 节点信息
struct DB50_Node {
    int16_t nodeno;
    int16_t nodetype_action_sp;
    int16_t action_l;
    int16_t action_h;
    float X;
    float Y;
    float Rz;
};

struct DB50_Agv {
    // VAR RETAIN
    DB50_Sys Sys;
    DB50_Status Status;
    // VAR
    DB50_Excute Excute;
    // VAR RETAIN
    bool Error[16];
    // VAR
    DB50_Data Data;
    // VAR RETAIN
    DB50_Node Node_Pre;
    DB50_Node Node_SP;
    // VAR
    int16_t Int_Node_No;
    // VAR RETAIN
    int16_t Int_Agv_No;
    int16_t Int_Step;
    int16_t Int_CRC_No;
    // VAR
    int32_t Dint_Move_dist;
    float Real_Tar_Speed;
    float Real_Speedset;
    int16_t Int_RCS_last_heartbeat;
    uint8_t Byte_RMT_last_heartbeat;
    bool Radar_Decel;
    bool Radar_Dstop;
};



void Tx_modbus_to_struct(const modbus_mapping_t* mapping, Tx_MODBUS_STRUCT& tx_agv);
void Rx_struct_to_modbus(const Rx_MODBUS_STRUCT& rx_agv, modbus_mapping_t* mapping, int base_addr = 0);


void calc_ang(
double in_degree_f,
double in_degree_b,
double in_dh,
double in_x,
double in_y,
double* ot_degree,
double* ot_r
);
void calc_line_endpoints(const QPointF& pos_mid, qreal orient_mid, qreal HH_in, QPointF* point_f_ot, QPointF* point_b_ot);
inline double mylimit(double ori_val,  double min_val, double max_val) ;








std::string get_local_id_from_ip();


// 边沿检测器类
class RisingEdgeDetector {
public:
    RisingEdgeDetector() : last_value(false) {}

    // 返回true表示本次检测到正跳变
    bool detect(bool current_value) {
        bool rising = (!last_value) && current_value;
        last_value = current_value;
        return rising;
    }
private:
    bool last_value;
};



}


#endif