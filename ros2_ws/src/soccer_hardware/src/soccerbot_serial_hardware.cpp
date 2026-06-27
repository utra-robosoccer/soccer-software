#include "soccer_hardware/soccerbot_serial_hardware.hpp"

#include <algorithm>
#include <cstring>
#include <unordered_map>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "soccer_hardware/wire_protocol.hpp"

namespace soccer_hardware
{

namespace
{
// Custom command-interface names for the MIT impedance gains. ros2_control
// allows arbitrary interface names beyond the position/velocity/effort triple.
constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

double param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double fallback)
{
  auto it = params.find(key);
  return it != params.end() ? std::stod(it->second) : fallback;
}
}  // namespace

hardware_interface::CallbackReturn SoccerbotSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto it = info_.hardware_parameters.find("serial_port");
      it != info_.hardware_parameters.end())
  {
    serial_port_ = it->second;
  }
  baud_rate_ = static_cast<int>(
    param_or(info_.hardware_parameters, "baud_rate", baud_rate_));
  watchdog_timeout_ = std::chrono::milliseconds(static_cast<int>(
    param_or(info_.hardware_parameters, "watchdog_timeout_ms",
             static_cast<double>(watchdog_timeout_.count()))));

  // Generic over N joints — size every buffer from the URDF.
  const size_t n = info_.joints.size();
  cmd_q_.assign(n, 0.0);
  cmd_qd_.assign(n, 0.0);
  cmd_kp_.assign(n, 0.0);   // safe default: zero impedance until a controller commands gains
  cmd_kd_.assign(n, 0.0);
  cmd_tau_.assign(n, 0.0);
  state_q_.assign(n, 0.0);
  state_qd_.assign(n, 0.0);
  state_tau_.assign(n, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("SoccerbotSerialHardware"),
    "Initialised serial hardware: %zu joints, port %s @ %d baud, watchdog %ld ms.",
    n, serial_port_.c_str(), baud_rate_, static_cast<long>(watchdog_timeout_.count()));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SoccerbotSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & j = info_.joints[i].name;
    si.emplace_back(j, hardware_interface::HW_IF_POSITION, &state_q_[i]);
    si.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &state_qd_[i]);
    si.emplace_back(j, hardware_interface::HW_IF_EFFORT, &state_tau_[i]);
  }
  if (!info_.sensors.empty()) {
    const std::string & s = info_.sensors[0].name;
    static const std::array<std::string, 10> names = {
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
SoccerbotSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & j = info_.joints[i].name;
    // Full MIT impedance tuple: q*, qd*, kp, kd, τ_ff.
    ci.emplace_back(j, hardware_interface::HW_IF_POSITION, &cmd_q_[i]);
    ci.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &cmd_qd_[i]);
    ci.emplace_back(j, HW_IF_KP, &cmd_kp_[i]);
    ci.emplace_back(j, HW_IF_KD, &cmd_kd_[i]);
    ci.emplace_back(j, hardware_interface::HW_IF_EFFORT, &cmd_tau_[i]);
  }
  return ci;
}

bool SoccerbotSerialHardware::open_port()
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
  // USB-CDC ignores the baud rate, but set it for non-CDC adapters.
  cfsetispeed(&tty, B1000000);
  cfsetospeed(&tty, B1000000);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
#else
  return false;  // serial not supported on this platform (e.g. Windows dev host)
#endif
}

void SoccerbotSerialHardware::close_port()
{
#if defined(__unix__) || defined(__APPLE__)
  if (fd_ >= 0) {
    ::close(fd_);
  }
#endif
  fd_ = -1;
}

hardware_interface::CallbackReturn SoccerbotSerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_rx_ = std::chrono::steady_clock::now();
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
  rx_accum_.clear();

  if (!open_port()) {
    // Absent/unplugged Master is non-fatal: enter ACTIVE in no-MCU mode (fd_<0).
    // read() is a no-op (no watchdog), write() drops frames, so the rest of the
    // ros2_control graph (perception, strategy, comms) keeps running.
    RCLCPP_WARN(
      rclcpp::get_logger("SoccerbotSerialHardware"),
      "Serial port %s unavailable — running in no-MCU mode (read/write are no-ops).",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(rclcpp::get_logger("SoccerbotSerialHardware"), "Serial link up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoccerbotSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool SoccerbotSerialHardware::poll_state_frame()
{
#if defined(__unix__) || defined(__APPLE__)
  uint8_t buf[1024];
  bool got_frame = false;
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; ++i) {
      if (buf[i] == 0x00) {
        // End of a COBS frame.
        std::vector<uint8_t> raw;
        if (!rx_accum_.empty() &&
            wire::cobs_decode(rx_accum_.data(), rx_accum_.size(), raw) &&
            raw.size() >= sizeof(wire::Header))
        {
          wire::Header hdr;
          std::memcpy(&hdr, raw.data(), sizeof(hdr));
          const uint16_t rx_crc = hdr.crc16;
          hdr.crc16 = 0;
          std::memcpy(raw.data(), &hdr, sizeof(hdr));  // zero crc field for check
          const uint16_t want = wire::crc16_ccitt(raw.data(), raw.size());
          if (want == rx_crc &&
              hdr.type == static_cast<uint16_t>(wire::Msg::MOTOR_STATE))
          {
            const uint8_t * p = raw.data() + sizeof(wire::Header);
            const size_t njoints = info_.joints.size();
            const size_t need =
              njoints * sizeof(wire::JointState) + sizeof(wire::BodyState);
            if (hdr.len >= need) {
              for (size_t k = 0; k < njoints; ++k) {
                wire::JointState js;
                std::memcpy(&js, p + k * sizeof(js), sizeof(js));
                state_q_[k] = js.q;
                state_qd_[k] = js.qd;
                state_tau_[k] = js.tau;
              }
              wire::BodyState bs;
              std::memcpy(&bs, p + njoints * sizeof(wire::JointState), sizeof(bs));
              // ros2_control IMU order: x,y,z,w. Wire order: w,x,y,z.
              imu_ = {bs.quat[1], bs.quat[2], bs.quat[3], bs.quat[0],
                      bs.gyro[0], bs.gyro[1], bs.gyro[2],
                      bs.accel[0], bs.accel[1], bs.accel[2]};
              got_frame = true;
            }
          }
        }
        rx_accum_.clear();
      } else {
        rx_accum_.push_back(buf[i]);
        if (rx_accum_.size() > 4096) {
          rx_accum_.clear();  // runaway / desync guard
        }
      }
    }
  }
  return got_frame;
#else
  return false;
#endif
}

void SoccerbotSerialHardware::send_command_frame()
{
#if defined(__unix__) || defined(__APPLE__)
  const uint16_t njoints = static_cast<uint16_t>(info_.joints.size());
  const size_t payload_len = sizeof(uint16_t) + njoints * sizeof(wire::JointMitCmd);

  std::vector<uint8_t> frame(sizeof(wire::Header) + payload_len);
  wire::Header hdr{};
  hdr.type = static_cast<uint16_t>(wire::Msg::MOTOR_CMD);
  hdr.seq = tx_seq_++;
  hdr.src = static_cast<uint8_t>(wire::Node::JETSON);
  hdr.dst = static_cast<uint8_t>(wire::Node::MASTER);
  hdr.ts_ms = 0;
  hdr.len = static_cast<uint16_t>(payload_len);
  hdr.flags = 0;
  hdr.crc16 = 0;
  std::memcpy(frame.data(), &hdr, sizeof(hdr));

  uint8_t * p = frame.data() + sizeof(wire::Header);
  std::memcpy(p, &njoints, sizeof(njoints));
  p += sizeof(njoints);
  for (uint16_t k = 0; k < njoints; ++k) {
    wire::JointMitCmd c{
      static_cast<float>(cmd_q_[k]), static_cast<float>(cmd_qd_[k]),
      static_cast<float>(cmd_kp_[k]), static_cast<float>(cmd_kd_[k]),
      static_cast<float>(cmd_tau_[k])};
    std::memcpy(p + k * sizeof(c), &c, sizeof(c));
  }

  // CRC over the whole frame with the crc field zeroed.
  const uint16_t crc = wire::crc16_ccitt(frame.data(), frame.size());
  std::memcpy(frame.data() + offsetof(wire::Header, crc16), &crc, sizeof(crc));

  const std::vector<uint8_t> cobs = wire::cobs_encode(frame.data(), frame.size());
  ::write(fd_, cobs.data(), cobs.size());
  const uint8_t delim = 0x00;
  ::write(fd_, &delim, 1);
#endif
}

hardware_interface::return_type SoccerbotSerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::OK;  // no-MCU mode
  }

  if (poll_state_frame()) {
    last_rx_ = std::chrono::steady_clock::now();
  }

  // Watchdog: if the Master has gone silent, fault so the controllers halt.
  if (std::chrono::steady_clock::now() - last_rx_ > watchdog_timeout_) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("SoccerbotSerialHardware"), steady_clock, 1000,
      "Master heartbeat lost (> %ld ms) — halting.",
      static_cast<long>(watchdog_timeout_.count()));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoccerbotSerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ >= 0) {
    send_command_frame();
  }
  return hardware_interface::return_type::OK;
}

}  // namespace soccer_hardware

PLUGINLIB_EXPORT_CLASS(
  soccer_hardware::SoccerbotSerialHardware, hardware_interface::SystemInterface)
