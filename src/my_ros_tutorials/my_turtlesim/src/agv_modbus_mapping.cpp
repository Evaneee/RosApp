#include "my_turtlesim/agv_modbus_mapping.hpp"
#include "my_turtlesim/my_turtle.hpp"

#include <sstream>
#include <iomanip>
#include "my_turtlesim/my_turtle.hpp"
#include <fcntl.h> // 文件顶部已包含
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <cstring>
#include <netdb.h>


#include <cstring>

namespace my_turtlesim {

void Tx_modbus_to_struct(const modbus_mapping_t* mapping, Tx_MODBUS_STRUCT& tx_agv, int base_addr) {
    const uint16_t* regs = mapping->tab_registers + base_addr;
    // General
    tx_agv.General.heartbeat      = regs[0];
    tx_agv.General.agvno          = regs[1];
    tx_agv.General.targetstation  = regs[2];
    tx_agv.General.tasktype       = regs[3];
    tx_agv.General.taskctrl       = regs[4];
    tx_agv.General.speedset_l     = regs[5];
    tx_agv.General.speedset_m     = regs[6];
    tx_agv.General.speedset_h     = regs[7];
    tx_agv.General.nodenum        = regs[8];
    tx_agv.General.agvctrl        = regs[9];
    tx_agv.General.reserve1       = regs[10];
    tx_agv.General.reserve2       = regs[11];
    tx_agv.General.reserve3       = regs[12];
    tx_agv.General.reserve4       = regs[13];
    tx_agv.General.reserve5       = regs[14];
    tx_agv.General.reserve6       = regs[15];
    tx_agv.General.reserve7       = regs[16];
    tx_agv.General.reserve8       = regs[17];
    tx_agv.General.reserve9       = regs[18];
    tx_agv.General.reserve10      = regs[19];

    // Seq
    for (int i = 0; i < 10; ++i) {
        int base = 20 + i * 10;
        tx_agv.Seq[i].nodeno             = regs[base + 0];
        tx_agv.Seq[i].nodetype_action_sp = regs[base + 1];
        tx_agv.Seq[i].action_l           = regs[base + 2];
        tx_agv.Seq[i].action_h           = regs[base + 3];
        // float X
        uint32_t x_raw = (uint32_t(regs[base + 4]) << 16) | regs[base + 5];
        std::memcpy(&tx_agv.Seq[i].X, &x_raw, sizeof(float));
        // float Y
        uint32_t y_raw = (uint32_t(regs[base + 6]) << 16) | regs[base + 7];
        std::memcpy(&tx_agv.Seq[i].Y, &y_raw, sizeof(float));
        // float Rz
        uint32_t rz_raw = (uint32_t(regs[base + 8]) << 16) | regs[base + 9];
        std::memcpy(&tx_agv.Seq[i].Rz, &rz_raw, sizeof(float));
    }
}

void Rx_struct_to_modbus(const Rx_MODBUS_STRUCT& rx_agv, modbus_mapping_t* mapping, int base_addr) {
    uint16_t* regs = mapping->tab_registers + base_addr;
    regs[0]  = rx_agv.heartbeat;
    regs[1]  = rx_agv.agvno;
    regs[2]  = rx_agv.station;
    regs[3]  = rx_agv.reserve;
    regs[4]  = rx_agv.nodetype_action_sp;
    regs[5]  = rx_agv.action_l;
    regs[6]  = rx_agv.action_h;
    regs[7]  = rx_agv.taskstate;
    regs[8]  = rx_agv.runstate;
    regs[9]  = rx_agv.execute_state1;
    regs[10] = rx_agv.execute_state2;
    regs[11] = rx_agv.speed;
    regs[12] = rx_agv.electricity;
    regs[13] = rx_agv.alarm;
    // float laser_x
    uint32_t lx;
    std::memcpy(&lx, &rx_agv.laser_x, sizeof(float));
    regs[14] = (lx >> 16) & 0xFFFF;
    regs[15] = lx & 0xFFFF;
    // float laser_y
    uint32_t ly;
    std::memcpy(&ly, &rx_agv.laser_y, sizeof(float));
    regs[16] = (ly >> 16) & 0xFFFF;
    regs[17] = ly & 0xFFFF;
    // float laser_a
    uint32_t la;
    std::memcpy(&la, &rx_agv.laser_a, sizeof(float));
    regs[18] = (la >> 16) & 0xFFFF;
    regs[19] = la & 0xFFFF;
    regs[20] = rx_agv.targetstation;
    regs[21] = rx_agv.reserve1;
    regs[22] = rx_agv.tasktype;
    regs[23] = rx_agv.enable;
    regs[24] = rx_agv.speedset_l;
    regs[25] = rx_agv.speedset_m;
    regs[26] = rx_agv.speedset_h;
    regs[27] = rx_agv.nodenum;
    regs[28] = rx_agv.reserve_26;
    regs[29] = rx_agv.reserve_27;
}



void Turtle::process_modbus_requests() {
    agv_state_machine();
    // 如果还没有客户端连接，尝试accept
    if (modbus_client_socket_fd_ < 0) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(modbus_server_socket_fd_, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        int retval = select(modbus_server_socket_fd_ + 1, &rfds, NULL, NULL, &tv);
        if (retval > 0 && FD_ISSET(modbus_server_socket_fd_, &rfds)) 
        {
            modbus_client_socket_fd_ = modbus_tcp_accept(modbus_ctx_, &modbus_server_socket_fd_);
            if (modbus_client_socket_fd_ >= 0) {
                int flags = fcntl(modbus_client_socket_fd_, F_GETFL, 0);
                fcntl(modbus_client_socket_fd_, F_SETFL, flags | O_NONBLOCK);
                //RCLCPP_INFO(nh_->get_logger(), "Turtle Modbus client connected");


                // 获取客户端IP和端口
                struct sockaddr_in addr;
                socklen_t addr_len = sizeof(addr);
                char ip_str[INET_ADDRSTRLEN] = {0};
                int port = 0;
                if (getpeername(modbus_client_socket_fd_, (struct sockaddr*)&addr, &addr_len) == 0) {
                    inet_ntop(AF_INET, &addr.sin_addr, ip_str, sizeof(ip_str));
                    port = ntohs(addr.sin_port);
                }

                // 获取本地端口并打印出来
                struct sockaddr_in local_addr;
                socklen_t local_addr_len = sizeof(local_addr);
                int local_port = 0;
                if (getsockname(modbus_client_socket_fd_, (struct sockaddr*)&local_addr, &local_addr_len) == 0) {
                    local_port = ntohs(local_addr.sin_port);
                }

                RCLCPP_INFO(nh_->get_logger(),
                    "Modbus client connected from %s:%d to local port %d",
                    ip_str, port, local_port);
            }
            else {
                RCLCPP_ERROR(nh_->get_logger(), "Failed to accept Modbus client connection");
            }
        }
        return;
    }

    // 用 select 检查是否有数据可读，避免阻塞
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(modbus_client_socket_fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0; // 立即返回

    //select函数的第一个参数只是检测的文件描述符号最大值，而不是总检测数量
    int retval = select(modbus_client_socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    if (retval > 0 && FD_ISSET(modbus_client_socket_fd_, &rfds)) {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbus_receive(modbus_ctx_, query);

        //打印收到的 Modbus 报文内容和乌龟名
        std::ostringstream oss;
        oss << "Turtle [" << real_name << "] modbus_receive [";
        for (int i = 0; i < rc; ++i) {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(query[i]);
            if (i != rc - 1) oss << " ";
        }
        oss << "]";
        //RCLCPP_INFO(nh_->get_logger(), "%s", oss.str().c_str());

        if (rc > 0) {

            //modbus_reply真正将接受到的内容更新到modbus_mapping_里面，只有写请求才会被处理
            modbus_reply(modbus_ctx_, query, rc, modbus_mapping_);
            //int reply_rc = modbus_reply(modbus_ctx_, query, rc, modbus_mapping_);
            Tx_modbus_to_struct(modbus_mapping_, db51_tcp_.Tx_Agv, 30);
            //RCLCPP_INFO(nh_->get_logger(), "Turtle [%s] modbus_reply 字节数=%d", real_name.c_str(), reply_rc);
        } else if (rc == -1) {
            if (errno == ECONNRESET || errno == ECONNABORTED) {
                close(modbus_client_socket_fd_);
                modbus_client_socket_fd_ = -1;
                RCLCPP_WARN(nh_->get_logger(), "Turtle Modbus client disconnected");
            }
        }
    }
}


std::string get_local_id_from_ip() {
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];
    std::string last_segment = "0";
    if (getifaddrs(&ifaddr) == -1) {
        return last_segment;
    }
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET &&
            std::strcmp(ifa->ifa_name, "lo") != 0) { // 排除回环
            getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                        host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
            std::string ip(host);
            size_t last_dot = ip.rfind('.');
            if (last_dot != std::string::npos) {
                last_segment = ip.substr(last_dot + 1);
                break;
            }
        }
    }
    freeifaddrs(ifaddr);
    return '_'+last_segment;
}



void calc_ang(
    double in_degree_f,
    double in_degree_b,
    double in_dh,
    double in_x,
    double in_y,
    double* ot_degree,
    double* ot_r
) {
    const double pi = 3.14159265;
    double ot_radian = 0.0;
    double in_radian_sf = in_degree_f / 180.0 * pi;
    double in_radian_sb = 0.0;

    if (std::abs(in_degree_f - in_degree_b) < 1E-3) {
        in_radian_sb = (in_degree_f + 1E-3) / 180.0 * pi;
    } else {
        in_radian_sb = in_degree_b / 180.0 * pi;
    }

    double x_o = in_dh / (std::tan(in_radian_sf) - std::tan(in_radian_sb));
    double y_o = -(std::tan(in_radian_sf) + std::tan(in_radian_sb)) / 2.0 * x_o;

    if (std::abs(x_o - in_x) > 1e-8) { // 避免除零
        ot_radian = -std::atan((in_y - y_o) / (in_x - x_o));
        if (ot_r) {
            *ot_r = (x_o - in_x) / std::cos(ot_radian);
        }
    } else {
        if (ot_r) {
            *ot_r = 0.0;
        }
    }

    if (ot_degree) {
        *ot_degree = ot_radian / pi * 180.0;
    }
}


void calc_line_endpoints(const QPointF& pos_mid, double orient_mid, double HH_in, QPointF* point_f_ot, QPointF* point_b_ot)
{
    qreal dx = (HH_in / 2.0) * std::cos(orient_mid);
    qreal dy = (HH_in / 2.0) * std::sin(orient_mid);
    if (point_f_ot) *point_f_ot = QPointF(pos_mid.x() + dx, pos_mid.y() + dy);
    if (point_b_ot) *point_b_ot = QPointF(pos_mid.x() - dx, pos_mid.y() - dy);
}

inline double mylimit(double ori_val, double min_val, double max_val) {
    if (ori_val > max_val) return max_val;
    if (ori_val < min_val) return min_val;
    return ori_val;
}


void Turtle::syn_agv_para()
{
    this->ang_f = mylimit(this->ang_f, -45, 45);
    this->ang_b = mylimit(this->ang_b, -45, 45);

    calc_line_endpoints(this->pos_, this->orient_, this->HH, &this->point_f, &this->point_b);

}





} // namespace my_turtlesim