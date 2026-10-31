#pragma once

#include <array>
#include <mujoco/mujoco.h>

namespace cartpole
{
// [x (m), x_dot (m/s), theta (rad), theta_dot (rad/s)].
// theta=0 points upward; positive theta leans right.
using State = std::array<double, 4>;

class Controller
{
public:
  // Linearize the supplied model at upright rest and solve discrete LQR once.
  void
  initialize(mjModel const* model);
  void
  reset();
  // Return horizontal cart force in N. Called once every physics step.
  double
  compute(State const& state, State const& reference, double time, double dt);

private:
  State gain_{};
  double timestep_ = 0;
};
} // namespace cartpole
