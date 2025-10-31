#include "mg6010_can_control/mg6010_hardware.hpp"

namespace mg6010_can_control
{

// Initialize the hardware interface
hardware_interface::CallbackReturn MG6010System::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

    ifname_ = info.hardware_parameters.at("can_interface");
    bitrate_ = std::stoi(info.hardware_parameters.at("bitrate"));

    // parse joints -> ids
    motors_.clear();
    for (const auto & joint : info.joints)
    {
        if (joint.parameters.find("id") == joint.parameters.end())
            return hardware_interface::CallbackReturn::ERROR;
        MotorCfg m;
        m.joint = joint.name;
        m.id = std::stoi(joint.parameters.at("id"));
        motors_.push_back(m);
    }

    size_t n = motors_.size();
    pos_deg_.assign(n, 0.0);
    vel_dps_.assign(n, 0.0);
    eff_amp_.assign(n, 0.0);
    cmd_vel_dps_.assign(n, 0.0);
    have_pos_.assign(n, false);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MG6010System::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < motors_.size(); ++i)
    {
        si.emplace_back(
            hardware_interface::StateInterface(
                motors_[i].joint, hardware_interface::HW_IF_POSITION, &pos_deg_[i]));
        si.emplace_back(
            hardware_interface::StateInterface(
                motors_[i].joint, hardware_interface::HW_IF_VELOCITY, &vel_dps_[i]));
        si.emplace_back(
            hardware_interface::StateInterface(
                motors_[i].joint, hardware_interface::HW_IF_EFFORT, &eff_amp_[i]));
    }
    return si;
}

std::vector<hardware_interface::CommandInterface> MG6010System::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < motors_.size(); ++i)
    {
        ci.emplace_back(
            hardware_interface::CommandInterface(
                motors_[i].joint, hardware_interface::HW_IF_VELOCITY, &cmd_vel_dps_[i]));
    }
    return ci;
}

hardware_interface::CallbackReturn MG6010System::on_activate(const rclcpp_lifecycle::State &)
{
    if (!open_can(ifname_, bitrate_))
    {
        RCLCPP_ERROR(logger_, "Failed to open CAN interface %s", ifname_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // turn on motors
    for (auto & m : motors_)
    {
        if (!motor_on(m.id))
        {
            RCLCPP_ERROR(logger_, "Failed to turn on motor ID %d", m.id);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MG6010System::on_deactivate(const rclcpp_lifecycle::State &)
{
    // close CAN socket
    if (sock_ >= 0)
    {
        close(sock_);
        sock_ = -1;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MG6010System::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // poll each motor state via 0x9C
  for (size_t i = 0; i < motors_.size(); ++i)
  {
    float T, vel, pos, iq;
    if (read_state_9C(motors_[i].id, T, vel, pos, iq))
    {
      vel_dps_[i] = vel;
      eff_amp_[i] = iq;
      // integrate deg using velocity (deg/s) * dt, fallback to reported pos if available
      double dt = period.seconds();
      if (!have_pos_[i]) {
        pos_deg_[i] = pos;
        have_pos_[i] = true;
      } else {
        pos_deg_[i] += vel * dt;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MG6010System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // A2 speed control in deg/s (protocol: 0.01 deg/s / LSB)
  for (size_t i = 0; i < motors_.size(); ++i)
  {
    if (!set_speed_A2(motors_[i].id, static_cast<float>(cmd_vel_dps_[i])))
      RCLCPP_WARN_THROTTLE(logger_, rclcpp::Clock(RCL_STEADY_TIME), 2000, "set_speed failed id=%d", motors_[i].id);
  }
  return hardware_interface::return_type::OK;
}

// ======== CAN helpers ========

bool MG6010System::open_can(const std::string & ifname, int /*bitrate*/)
{
  // 假设用户已通过 `ip link` 配置好 bitrate；这里直接打开 raw socket
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) return false;

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ-1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) return false;

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) return false;

  // 非阻塞可选：fcntl(sock_, F_SETFL, O_NONBLOCK);
  return true;
}

bool MG6010System::send_cmd(uint32_t can_id, const uint8_t *data)
{
  struct can_frame f{};
  f.can_id  = can_id;
  f.can_dlc = 8;
  std::memcpy(f.data, data, 8);
  return write(sock_, &f, sizeof(f)) == sizeof(f);
}

bool MG6010System::recv_frame(struct can_frame & frame, int timeout_ms)
{
  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(sock_, &readset);
  struct timeval tv{ timeout_ms/1000, (timeout_ms%1000)*1000 };
  int rv = select(sock_+1, &readset, nullptr, nullptr, &tv);
  if (rv <= 0) return false;
  ssize_t n = read(sock_, &frame, sizeof(frame));
  return n == sizeof(frame);
}

// ======== MG protocol ========
// id: 1..32 ; TX ID = 0x140 + id
// A2 speed: data[0]=0xA2, data[4..7] int32 (0.01 deg/s per LSB, little-endian)
// 9C read state: request data[0]=0x9C ; reply same id & cmd

bool MG6010System::motor_on(int id)
{
  uint8_t d[8] = {0};
  d[0] = 0x88; // Motor On
  return send_cmd(0x140 + id, d);
}

bool MG6010System::set_speed_A2(int id, float vel_dps)
{
  int32_t spd = static_cast<int32_t>(vel_dps * 100.0f); // 0.01 deg/s
  uint8_t d[8] = {0};
  d[0] = 0xA2;
  std::memcpy(&d[4], &spd, 4); // little endian
  return send_cmd(0x140 + id, d);
}

bool MG6010System::read_state_9C(int id, float & temp_c, float & vel_dps, float & pos_deg, float & iq_a)
{
  uint8_t d[8] = {0};
  d[0] = 0x9C;
  if (!send_cmd(0x140 + id, d)) return false;

  struct can_frame fr{};
  // 简单循环读取直到匹配
  const int max_try = 10;
  for (int i=0; i<max_try; ++i)
  {
    if (!recv_frame(fr, 5)) continue;
    if ((fr.can_id == (uint32_t)(0x140 + id)) && fr.data[0] == 0x9C)
    {
      int8_t  temp = (int8_t)fr.data[1];
      int16_t iq   = *(int16_t*)&fr.data[2];     // 2048 LSB ≈ 33 A
      int16_t spd  = *(int16_t*)&fr.data[4];     // 1 deg/s per LSB (协议中有 A2 下发单位 0.01deg/s，反馈通常 1 deg/s LSB，按文档为准)
      uint16_t enc = *(uint16_t*)&fr.data[6];    // 0..65535

      temp_c  = (float)temp;
      iq_a    = (float)iq * (33.0f / 2048.0f);
      vel_dps = (float)spd;          // 如你的手册是 0.1/0.01 度每秒，请按需改系数
      // 这里无法直接得多圈角度，先将编码器映射到 0..360
      pos_deg = (float)enc * (360.0f / 65536.0f);
      return true;
    }
  }
  return false;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mg6010_can_control::MG6010System, hardware_interface::SystemInterface)
