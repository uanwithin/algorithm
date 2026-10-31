#pragma once

#include "controller.hh"

#include <mujoco/mujoco.h>

#include <memory>

namespace cartpole
{
class Simulation
{
public:
  explicit Simulation(double target_x = 0);
  void
  reset();
  void
  step(double manual_force = 0);
  State
  state() const;
  mjModel const*
  model() const
  {
    return model_.get();
  }
  mjData*
  data()
  {
    return data_.get();
  }
  mjData const*
  data() const
  {
    return data_.get();
  }
  double
  timestep() const
  {
    return model_->opt.timestep;
  }
  double
  force() const
  {
    return data_->ctrl[0];
  }
  double
  target() const
  {
    return reference_[0];
  }

private:
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr,
                                                             mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr,
                                                          mj_deleteData};
  Controller controller_;
  State reference_{};
  int initial_key_ = -1;
};

int
run_viewer(Simulation& simulation);
} // namespace cartpole
