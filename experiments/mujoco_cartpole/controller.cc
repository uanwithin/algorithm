#include "controller.hh"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>

namespace cartpole
{
void
Controller::initialize(mjModel const* model)
{}

void
Controller::reset()
{
  // Static state feedback has no history. Keep the gain computed at startup.
}

double
Controller::compute(State const& /*state*/,
                    State const& /*reference*/,
                    double /*time*/,
                    double /*dt*/)
{
  return 0.0;
}
} // namespace cartpole
