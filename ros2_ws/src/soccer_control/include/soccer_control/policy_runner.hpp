// PolicyRunner — a thin wrapper around the exported RL residual policy.
//
// In production this loads the ONNX policy exported from Isaac Lab (sim/export)
// and runs it through ONNX Runtime / TensorRT. Here it is a header-only stub:
// with no policy file it returns a ZERO residual, which makes the controller
// fall back to *pure MPC* — the safe default before a policy is trained
// (blueprint §4, §10). The observation/return contract matches the sim env so
// swapping the stub for real inference does not change the controller.
#ifndef SOCCER_CONTROL__POLICY_RUNNER_HPP_
#define SOCCER_CONTROL__POLICY_RUNNER_HPP_

#include <string>
#include <vector>

namespace soccer_control
{

class PolicyRunner
{
public:
  // Loads the policy. Returns false (and stays in "zero residual" mode) when the
  // path is empty or the file is missing — never throws into the RT loop.
  bool load(const std::string & onnx_path)
  {
    onnx_path_ = onnx_path;
    loaded_ = !onnx_path.empty();
    // TODO(perception/ml): create the ONNX Runtime session / TensorRT engine here.
    return loaded_;
  }

  bool loaded() const { return loaded_; }

  // obs layout MUST match the Isaac Lab task observation:
  //   [ q, qd, q_ref, qd_ref, imu_gyro_z, command_bearing ]
  // Returns the residual joint delta Δq (radians). Stub => 0.
  double residual(const std::vector<float> & obs) const
  {
    (void)obs;
    if (!loaded_) {
      return 0.0;  // pure MPC fallback
    }
    // TODO(perception/ml): float delta = session_.run(obs)[0]; return delta;
    return 0.0;
  }

private:
  std::string onnx_path_;
  bool loaded_{false};
};

}  // namespace soccer_control

#endif  // SOCCER_CONTROL__POLICY_RUNNER_HPP_
