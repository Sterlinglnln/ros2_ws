#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

namespace mg6010_can_control
{

struct MotorCfg
{
  int id = 1;                // 1..32
  std::string joint;         // joint name
};

class MG6010System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MG6010System)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & prev) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN helpers
  bool open_can(const std::string & ifname, int bitrate);
  bool send_cmd(uint32_t can_id, const uint8_t *data);
  bool recv_frame(struct can_frame & frame, int timeout_ms);

  // MG commands
  bool motor_on(int id);
  bool read_state_9C(int id, float & temp_c, float & vel_dps, float & pos_deg, float & iq_a);
  bool set_speed_A2(int id, float vel_dps);

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("MG6010System");
  int sock_ = -1;
  std::string ifname_ = "can0";
  int bitrate_ = 1000000;

  // joints
  std::vector<MotorCfg> motors_;
  std::vector<double> pos_deg_;   // position (deg, multi-turn estimate)
  std::vector<double> vel_dps_;   // velocity (deg/s)
  std::vector<double> eff_amp_;   // effort -> iq(A) (approx)
  std::vector<double> cmd_vel_dps_; // command velocity (deg/s)

  // integrate position
  std::vector<bool> have_pos_;
};

} // namespace mg6010_can_control
