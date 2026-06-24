#include "soccer_hardware/minibot_serial_hardware.hpp"

#include <algorithm>
#include <cstring>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace soccer_hardware
{

uint16_t crc16_ccitt(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

hardware_interface::CallbackReturn MinibotSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? it->second : def;
  };
  serial_port_ = get("serial_port", "/dev/ttyACM0");
  baud_rate_ = std::stoi(get("baud_rate", "1000000"));
  watchdog_timeout_ =
    std::chrono::milliseconds(std::stoi(get("watchdog_timeout_ms", "100")));

  RCLCPP_INFO(
    rclcpp::get_logger("MinibotSerialHardware"),
    "Real hardware on %s @ %d baud, watchdog %ld ms",
    serial_port_.c_str(), baud_rate_, static_cast<long>(watchdog_timeout_.count()));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MinibotSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  const std::string joint = info_.joints[0].name;
  si.emplace_back(joint, hardware_interface::HW_IF_POSITION, &pos_);
  si.emplace_back(joint, hardware_interface::HW_IF_VELOCITY, &vel_);
  si.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &eff_);
  if (!info_.sensors.empty()) {
    const std::string s = info_.sensors[0].name;
    const std::array<std::string, 10> names = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
    for (size_t i = 0; i < names.size(); ++i) {
      si.emplace_back(s, names[i], &imu_[i]);
    }
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
MinibotSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  const std::string joint = info_.joints[0].name;
  ci.emplace_back(joint, hardware_interface::HW_IF_POSITION, &cmd_pos_);
  ci.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &cmd_eff_);
  return ci;
}

bool MinibotSerialHardware::open_port()
{
#if defined(__unix__) || defined(__APPLE__)
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close_port();
    return false;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B1000000);
  cfsetospeed(&tty, B1000000);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
#else
  return false;  // serial not supported on this platform
#endif
}

void MinibotSerialHardware::close_port()
{
#if defined(__unix__) || defined(__APPLE__)
  if (fd_ >= 0) {
    ::close(fd_);
  }
#endif
  fd_ = -1;
}

hardware_interface::CallbackReturn MinibotSerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialise last_rx_ so the watchdog has a valid baseline regardless of
  // whether the port opens. The fd_ < 0 guard in read() prevents the watchdog
  // from running until a real link is established anyway.
  last_rx_ = std::chrono::steady_clock::now();
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};

  if (!open_port()) {
    // Absent or unplugged MCU is a non-fatal condition: the hardware plugin
    // enters ACTIVE with fd_ = -1. read() returns OK immediately (no-op, no
    // watchdog), write() silently drops commands. The ros2_control graph keeps
    // running so all other nodes (perception, strategy, comms) stay up.
    // ros2_control 4.x throws std::runtime_error if on_activate returns ERROR,
    // killing the entire controller_manager — returning SUCCESS avoids that.
    RCLCPP_WARN(
      rclcpp::get_logger("MinibotSerialHardware"),
      "Serial port %s not available — running in no-MCU mode (read/write are no-ops).",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(rclcpp::get_logger("MinibotSerialHardware"), "Serial link up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MinibotSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MinibotSerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Guard: read() must not run before on_activate() opens the port.
  // ros2_control 4.x may call read() during initialisation before the
  // hardware lifecycle reaches ACTIVE; returning OK with stale (zero) state
  // is safe because no controller is commanding the joint yet.
  if (fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

#if defined(__unix__) || defined(__APPLE__)
  // Wire format mirrors soccer-firmware: header + MOTOR_STATE "<ffffIH"
  // (pos, vel, eff, temp, ts_us, crc). A real implementation reassembles frames
  // across reads; here we parse whole frames for clarity.
  struct __attribute__((packed)) MotorState {
    float pos; float vel; float eff; float temp; uint32_t ts_us; uint16_t crc;
  } state{};
  ssize_t n = (fd_ >= 0) ? ::read(fd_, &state, sizeof(state)) : -1;
  if (n == static_cast<ssize_t>(sizeof(state))) {
    const uint16_t want = crc16_ccitt(
      reinterpret_cast<const uint8_t *>(&state), sizeof(state) - sizeof(uint16_t));
    if (want == state.crc) {
      pos_ = state.pos;
      vel_ = state.vel;
      eff_ = state.eff;
      last_rx_ = std::chrono::steady_clock::now();
    }
  }
#endif

  // Watchdog: if the MCU has gone silent, fault so the controllers halt.
  const auto silent = std::chrono::steady_clock::now() - last_rx_;
  if (silent > watchdog_timeout_) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("MinibotSerialHardware"), steady_clock, 1000,
      "MCU heartbeat lost (> %ld ms) — halting.",
      static_cast<long>(watchdog_timeout_.count()));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MinibotSerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
#if defined(__unix__) || defined(__APPLE__)
  // MOTOR_CMD "<fffff": (q_target, qd_target, kp, kd, tau_ff) — MIT-style impedance
  // command the MCU runs at 1 kHz. We send q_target + feed-forward effort; the
  // gains are configured on the MCU.
  struct __attribute__((packed)) MotorCmd {
    float q; float qd; float kp; float kd; float tau_ff; uint16_t crc;
  } cmd{static_cast<float>(cmd_pos_), 0.0f, 0.0f, 0.0f,
        static_cast<float>(cmd_eff_), 0};
  cmd.crc = crc16_ccitt(
    reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd) - sizeof(uint16_t));
  if (fd_ >= 0) {
    ssize_t w = ::write(fd_, &cmd, sizeof(cmd));
    (void)w;
  }
#endif
  return hardware_interface::return_type::OK;
}

}  // namespace soccer_hardware

PLUGINLIB_EXPORT_CLASS(
  soccer_hardware::MinibotSerialHardware, hardware_interface::SystemInterface)
