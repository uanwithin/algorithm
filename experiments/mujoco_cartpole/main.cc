#include "simulation.hh"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

int
main(int argc, char** argv)
{
  try
  {
    bool headless = false;
    double seconds = 5;
    double force = 0;
    double target = 0;
    bool headless_option = false;
    for(int i = 1; i < argc; ++i)
    {
      std::string_view const arg(argv[i]);
      if(arg == "--help")
      {
        std::cout << "Usage: " << argv[0]
                  << " [--target METERS] [--headless [--seconds SECONDS] "
                     "[--force NEWTONS]]\n"
                  << "Discrete LQR balances the pole and tracks --target "
                     "(default 0 m).\n";
        return EXIT_SUCCESS;
      }
      if(arg == "--headless")
      {
        headless = true;
        continue;
      }
      if(arg != "--seconds" && arg != "--force" && arg != "--target")
        throw std::runtime_error("Unknown argument: " + std::string(arg));
      if(++i == argc)
        throw std::runtime_error("Missing value for " + std::string(arg));
      std::string const token(argv[i]);
      std::size_t end = 0;
      double const value = std::stod(token, &end);
      if(end != token.size() || !std::isfinite(value))
        throw std::runtime_error("Expected a finite number for " +
                                 std::string(arg));
      if(arg == "--target")
        target = value;
      else
      {
        headless_option = true;
        if(arg == "--seconds")
          seconds = value;
        else
          force = value;
      }
    }
    if(seconds <= 0)
      throw std::runtime_error("--seconds must be positive");
    if(headless_option && !headless)
      throw std::runtime_error("--seconds and --force require --headless");
    cartpole::Simulation simulation(target);
    if(!headless)
      return cartpole::run_viewer(simulation);
    std::cout << "time_s,x_m,x_dot_m_s,theta_rad,theta_dot_rad_s,force_N\n"
              << std::fixed << std::setprecision(6);
    auto print = [&]()
    {
      std::cout << simulation.data()->time;
      for(double value : simulation.state())
        std::cout << ',' << value;
      std::cout << ',' << simulation.force() << '\n';
    };
    print();
    int since_print = 0;
    while(simulation.data()->time + 0.5 * simulation.timestep() < seconds)
    {
      simulation.step(force);
      if(++since_print == 50)
      {
        print();
        since_print = 0;
      }
    }
    if(since_print)
      print();
    return EXIT_SUCCESS;
  }
  catch(std::exception const& error)
  {
    std::cerr << "Cart-pole: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
