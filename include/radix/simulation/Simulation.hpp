#ifndef RADIX_SIMULATION_SIMULATION_HPP
#define RADIX_SIMULATION_SIMULATION_HPP

#include <type_traits>

#include <radix/core/types/TimeDelta.hpp>

namespace radix {
namespace simulation {

class World;

class Simulation {
protected:
  World &world;

public:
  Simulation(World &w) : world(w) {}

  virtual const char* getName() const = 0;
  virtual bool runsBefore(const Simulation&) { return false; }
  virtual bool runsAfter(const Simulation&) { return false; }
  virtual void update(TDelta dtime) = 0;
};

} /* namespace simulation */
} /* namespace radix */

#endif /* RADIX_SIMULATION_SIMULATION_HPP */
