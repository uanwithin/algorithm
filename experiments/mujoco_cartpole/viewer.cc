#include "simulation.hh"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace cartpole
{
namespace
{
struct WindowSystem
{
  WindowSystem()
  {
    glfwSetErrorCallback(
        [](int code, char const* message)
        { std::cerr << "GLFW " << code << ": " << message << '\n'; });
    if(glfwInit() == 0)
    {
      throw std::runtime_error(
          "Cannot open graphics; use --headless without a display");
    }
  }
  ~WindowSystem()
  {
    glfwTerminate();
  }
};

struct Viewer
{
  Simulation& simulation;
  mjvCamera camera{};
  mjvOption option{};
  mjvScene scene{};
  mjrContext context{};
  bool paused = true;
  bool single_step = false;
  bool reset_clock = false;

  explicit Viewer(Simulation& sim)
    : simulation(sim)
  {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(simulation.model(), &scene, 1000);
    mjr_makeContext(simulation.model(), &context, mjFONTSCALE_150);
    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = mj_name2id(simulation.model(), mjOBJ_CAMERA, "side");
    scene.flags[mjRND_SHADOW] = 0;
    scene.flags[mjRND_REFLECTION] = 0;
  }
  ~Viewer()
  {
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
  }

  mjvGeom&
  geometry(int type, double x, double y, double z, float const* color)
  {
    if(scene.ngeom == scene.maxgeom)
    {
      throw std::runtime_error("Viewer geometry capacity exceeded");
    }
    auto& geom = scene.geoms[scene.ngeom++];
    mjtNum size[3]{0.01, 0.01, 0.01};
    mjtNum position[3]{x, y, z};
    mjtNum rotation[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    mjv_initGeom(&geom, type, size, position, rotation, color);
    return geom;
  }

  void
  line(double x1,
       double z1,
       double x2,
       double z2,
       float const* color,
       int type = mjGEOM_LINE)
  {
    auto& geom = geometry(type, 0, 0, 0, color);
    mjtNum from[3]{x1, -0.1, z1};
    mjtNum to[3]{x2, -0.1, z2};
    mjv_connector(&geom, type, type == mjGEOM_ARROW ? 0.008 : 1.5, from, to);
  }

  void
  label(double x, double z, char const* text, float const* color)
  {
    auto& geom = geometry(mjGEOM_LABEL, x, -0.12, z, color);
    std::snprintf(geom.label, sizeof(geom.label), "%s", text);
  }

  void
  decorations()
  {
    float const grey[4]{0.55F, 0.60F, 0.57F, 1};
    float const gold[4]{0.82F, 0.57F, 0.23F, 1};
    for(int i = -10; i <= 10; ++i)
    {
      double const x = 0.1 * i;
      line(x, -0.04, x, i % 5 == 0 ? -0.075 : -0.06, grey);
      if(i % 5 == 0)
      {
        char text[20];
        std::snprintf(text, sizeof(text), "%.1f", x);
        label(x, -0.115, text, grey);
      }
    }
    label(1.0, -0.18, "x / m", grey);
    for(int i = 0; i < 20; ++i)
    {
      line(simulation.target(),
           0.04 + i * 0.045,
           simulation.target(),
           0.065 + i * 0.045,
           gold);
    }
    double const x = simulation.state()[0];
    double const force = simulation.force();
    if(std::abs(force) > 1e-6)
    {
      line(x, 0.075, x + 0.025 * force, 0.075, gold, mjGEOM_ARROW);
    }
    char text[40];
    std::snprintf(text, sizeof(text), "%+.2f N", force);
    label(x - 0.23, 0.11, text, gold);
  }
};

void
keyboard(
    GLFWwindow* window, int key, int /*unused*/, int action, int /*unused*/)
{
  if(action != GLFW_PRESS)
  {
    return;
  }
  auto& viewer = *static_cast<Viewer*>(glfwGetWindowUserPointer(window));
  if(key == GLFW_KEY_ESCAPE)
  {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
  if(key == GLFW_KEY_SPACE)
  {
    viewer.paused = !viewer.paused;
    viewer.reset_clock = true;
  }
  if(key == GLFW_KEY_N && viewer.paused)
  {
    viewer.single_step = true;
  }
  if(key == GLFW_KEY_R)
  {
    viewer.simulation.reset();
    viewer.paused = true;
    viewer.single_step = false;
    viewer.reset_clock = true;
  }
}
} // namespace

int
run_viewer(Simulation& simulation)
{
  WindowSystem window_system;
  std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window(
      glfwCreateWindow(1200,
                       800,
                       "MuJoCo Cart-Pole | LQR Playground",
                       nullptr,
                       nullptr),
      glfwDestroyWindow);
  if(!window)
  {
    throw std::runtime_error("Cannot create OpenGL window; try --headless");
  }
  glfwMakeContextCurrent(window.get());
  glfwSwapInterval(1);
  // Destruction order keeps the OpenGL context alive until Viewer releases it.
  Viewer viewer(simulation);
  glfwSetWindowUserPointer(window.get(), &viewer);
  glfwSetKeyCallback(window.get(), keyboard);
  double previous = glfwGetTime();
  double accumulated = 0;
  while(glfwWindowShouldClose(window.get()) == 0)
  {
    glfwPollEvents();
    double const now = glfwGetTime();
    if(viewer.paused || viewer.reset_clock)
    {
      accumulated = 0;
    }
    else
    {
      accumulated += std::min(now - previous, 0.1);
    }
    previous = now;
    viewer.reset_clock = false;
    // Held keys add a disturbance to the controller; losing focus releases it.
    double manual = 0;
    if(glfwGetWindowAttrib(window.get(), GLFW_FOCUSED) != 0)
    {
      manual =
          2.0 * (static_cast<int>(glfwGetKey(window.get(), GLFW_KEY_RIGHT) ==
                                  GLFW_PRESS) -
                 static_cast<int>(glfwGetKey(window.get(), GLFW_KEY_LEFT) ==
                                  GLFW_PRESS));
    }
    if(viewer.single_step)
    {
      simulation.step(manual);
      viewer.single_step = false;
    }
    while(accumulated >= simulation.timestep())
    {
      simulation.step(manual);
      accumulated -= simulation.timestep();
    }
    mj_forward(simulation.model(), simulation.data());
    mjrRect viewport{};
    glfwGetFramebufferSize(window.get(), &viewport.width, &viewport.height);
    if((viewport.width != 0) && (viewport.height != 0))
    {
      mjv_updateScene(simulation.model(),
                      simulation.data(),
                      &viewer.option,
                      nullptr,
                      &viewer.camera,
                      mjCAT_ALL,
                      &viewer.scene);
      viewer.decorations();
      mjr_render(viewport, &viewer.scene, &viewer.context);
      auto const state = simulation.state();
      char status[320];
      std::snprintf(status,
                    sizeof(status),
                    "%s   t=%.2f s   target=%.2f m\nx=%+.3f m   v=%+.3f m/s\n"
                    "theta=%+.3f rad   omega=%+.3f rad/s   F=%+.2f N",
                    viewer.paused ? "PAUSED" : "RUNNING",
                    simulation.data()->time,
                    simulation.target(),
                    state[0],
                    state[1],
                    state[2],
                    state[3],
                    simulation.force());
      mjr_overlay(mjFONT_NORMAL,
                  mjGRID_TOPLEFT,
                  viewport,
                  status,
                  nullptr,
                  &viewer.context);
      mjr_overlay(mjFONT_NORMAL,
                  mjGRID_BOTTOMLEFT,
                  viewport,
                  "Space: play/pause | R: reset | N: single step | Esc: exit\n"
                  "Hold Left/Right: -/+2 N disturbance | LQR balancing active",
                  nullptr,
                  &viewer.context);
      if(std::abs(state[0]) > 1.0)
      {
        mjr_overlay(mjFONT_NORMAL,
                    mjGRID_TOPRIGHT,
                    viewport,
                    "Cart outside visible rail. Press R to reset.",
                    nullptr,
                    &viewer.context);
      }
      glfwSwapBuffers(window.get());
    }
    glfwWaitEventsTimeout(0.001);
  }
  return EXIT_SUCCESS;
}
} // namespace cartpole
