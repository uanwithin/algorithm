#include "simulation.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void
require(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

void
verify_closed_loop(double target, double disturbance, bool mirrored = false)
{
  cartpole::Simulation sim(target);
  if(mirrored)
  {
    sim.data()->qpos[0] *= -1;
    sim.data()->qpos[1] *= -1;
    mj_forward(sim.model(), sim.data());
  }
  double peak_force = 0;
  double peak_angle = 0;
  double last_outside = 0;
  for(int i = 0; i < 4000; ++i)
  {
    // Same input path as the arrow keys: a 250 ms, +/-2 N cart disturbance.
    double const time = sim.data()->time;
    sim.step(time >= 2 && time < 2.25 ? disturbance : 0);
    auto const state = sim.state();
    peak_force = std::max(peak_force, std::abs(sim.force()));
    peak_angle = std::max(peak_angle, std::abs(state[2]));
    require(std::abs(state[2]) < 0.35, "Closed loop keeps pole near upright");
    require(std::abs(state[0]) < 1, "Closed loop stays on visible rail");
    require(std::abs(sim.force()) <= 10, "Closed loop respects force limit");
    if(std::abs(state[0] - target) > 0.005 || std::abs(state[1]) > 0.01 ||
       std::abs(state[2]) > 0.005 || std::abs(state[3]) > 0.01)
      last_outside = sim.data()->time;
  }
  auto const final = sim.state();
  require(std::abs(final[0] - target) < 1e-4 && std::abs(final[1]) < 1e-3 &&
              std::abs(final[2]) < 1e-4 && std::abs(final[3]) < 1e-3,
          "LQR converges to target after disturbance is released");
  require(last_outside < 6, "LQR settles within six seconds");
  std::cout << "PASS: LQR target=" << target
            << " m, disturbance=" << disturbance << " N, mirrored=" << mirrored
            << ", settled by " << last_outside + sim.timestep()
            << " s, peak force=" << peak_force
            << " N, peak angle=" << peak_angle << " rad\n";
  // Reset must reproduce the same feedback on the same initial state.
  sim.reset();
  sim.step();
  auto const first_step = sim.state();
  double const first_force = sim.force();
  sim.reset();
  sim.step();
  require(sim.state() == first_step && sim.force() == first_force,
          "Controller reset is reproducible");
}
} // namespace

int
main()
{
  try
  {
    cartpole::Simulation sim;
    auto const initial = sim.state();
    require(std::abs(initial[0] - 0.2) < 1e-12 &&
                std::abs(initial[2] + 0.05) < 1e-12,
            "Initial state mapping");
    // Checks below exercise MuJoCo directly so they remain valid after you implement LQR.
    mj_resetData(sim.model(), sim.data());
    mj_forward(sim.model(), sim.data());
    for(int i = 0; i < 500; ++i)
    {
      mj_step(sim.model(), sim.data());
    }
    for(double value : sim.state())
    {
      require(std::abs(value) < 1e-10, "Upright equilibrium");
    }

    sim.data()->ctrl[0] = 1;
    mj_forward(sim.model(), sim.data());
    require(sim.data()->qacc[0] > 0 && sim.data()->qacc[1] < 0,
            "Positive force direction");
    require(std::abs(sim.data()->qfrc_actuator[0] - 1) < 1e-12 &&
                sim.data()->qfrc_actuator[1] == 0,
            "Only cart receives actuator force");
    sim.data()->ctrl[0] = 100;
    mj_forward(sim.model(), sim.data());
    require(std::abs(sim.data()->actuator_force[0] - 10) < 1e-12,
            "Positive force limit");
    sim.data()->ctrl[0] = -100;
    mj_forward(sim.model(), sim.data());
    require(std::abs(sim.data()->actuator_force[0] + 10) < 1e-12,
            "Negative force limit");

    mj_resetData(sim.model(), sim.data());
    sim.data()->qpos[1] = 0.01;
    mj_forward(sim.model(), sim.data());
    require(sim.data()->qacc[1] > 0, "Gravity destabilizes upright pole");
    int const bob = mj_name2id(sim.model(), mjOBJ_GEOM, "bob");
    require(sim.data()->geom_xpos[static_cast<ptrdiff_t>(3 * bob)] > 0,
            "Positive angle leans right");
    mj_energyPos(sim.model(), sim.data());
    mj_energyVel(sim.model(), sim.data());
    double const energy = sim.data()->energy[0] + sim.data()->energy[1];
    for(int i = 0; i < 2500; ++i)
    {
      mj_step(sim.model(), sim.data());
    }
    require(std::abs(sim.state()[2]) > 0.05, "Open-loop pole falls");
    for(double value : sim.state())
    {
      require(std::isfinite(value), "Finite state after 5 seconds");
    }
    mj_forward(sim.model(), sim.data());
    mj_energyPos(sim.model(), sim.data());
    mj_energyVel(sim.model(), sim.data());
    require(std::abs(sim.data()->energy[0] + sim.data()->energy[1] - energy) <
                1e-6,
            "Unforced frictionless system conserves energy");

    sim.reset();
    require(sim.state() == initial && sim.data()->time == 0 && sim.force() == 0,
            "Reset state/time/force");
    bool rejected = false;
    try
    {
      sim.step(std::numeric_limits<double>::quiet_NaN());
    }
    catch(std::runtime_error const&)
    {
      rejected = true;
    }
    require(rejected, "Reject non-finite control");
    std::cout << "PASS: state convention, equilibrium, force direction/limits, "
                 "gravity, energy, reset, invalid input\n";
    verify_closed_loop(0, 0);
    verify_closed_loop(0.3, 0);
    verify_closed_loop(-0.3, 0);
    verify_closed_loop(0, 2);
    verify_closed_loop(0, -2);
    verify_closed_loop(0, 0, true);
    return EXIT_SUCCESS;
  }
  catch(std::exception const& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
