#include "simulation.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace cartpole
{
Simulation::Simulation(double target_x)
  : reference_{target_x, 0, 0, 0}
{
  if(!std::isfinite(target_x))
    throw std::runtime_error("Target position must be finite");
  if(mj_version() != mjVERSION_HEADER)
    throw std::runtime_error("MuJoCo header/library version mismatch");
  char error[1024]{};
  model_.reset(mj_loadXML(CARTPOLE_MODEL, nullptr, error, sizeof(error)));
  if(!model_)
    throw std::runtime_error(std::string("Cannot load cartpole.xml: ") + error);
  int const cart = mj_name2id(model(), mjOBJ_JOINT, "cart_x");
  int const pole = mj_name2id(model(), mjOBJ_JOINT, "pole_angle");
  int const motor = mj_name2id(model(), mjOBJ_ACTUATOR, "cart_force");
  initial_key_ = mj_name2id(model(), mjOBJ_KEY, "initial");
  if(model_->nq != 2 || model_->nv != 2 || model_->nu != 1 || cart != 0 ||
     pole != 1 || motor != 0 || initial_key_ < 0 ||
     model_->jnt_type[cart] != mjJNT_SLIDE ||
     model_->jnt_type[pole] != mjJNT_HINGE ||
     model_->actuator_trnid[0] != cart || model_->actuator_gear[0] != 1 ||
     !model_->actuator_ctrllimited[0])
    throw std::runtime_error(
        "Expected supplied two-joint cart-pole with force-limited cart motor");
  data_.reset(mj_makeData(model()));
  if(!data_)
    throw std::runtime_error("Cannot allocate MuJoCo state");
  controller_.initialize(model());
  reset();
}

State
Simulation::state() const
{
  return {data_->qpos[0], data_->qvel[0], data_->qpos[1], data_->qvel[1]};
}

void
Simulation::reset()
{
  mj_resetDataKeyframe(model(), data(), initial_key_);
  controller_.reset();
  mj_forward(model(), data());
}

void
Simulation::step(double manual_force)
{
  double const command =
      controller_.compute(state(), reference_, data_->time, timestep()) +
      manual_force;
  if(!std::isfinite(command))
    throw std::runtime_error(
        "Controller/manual input returned a non-finite force");
  data_->ctrl[0] = std::clamp(command,
                              model_->actuator_ctrlrange[0],
                              model_->actuator_ctrlrange[1]);
  mj_step(model(), data());
  for(double value : state())
    if(!std::isfinite(value))
      throw std::runtime_error("Simulation produced a non-finite state");
  // MuJoCo can auto-reset invalid dynamics; make that visible instead of hiding it.
  if(data_->warning[mjWARN_BADQPOS].number ||
     data_->warning[mjWARN_BADQVEL].number ||
     data_->warning[mjWARN_BADQACC].number ||
     data_->warning[mjWARN_BADCTRL].number)
    throw std::runtime_error(
        "MuJoCo reported invalid dynamics; reset and check the controller");
}
} // namespace cartpole
