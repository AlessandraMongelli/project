#ifndef FLOCK_H
#define FLOCK_H
#include "boid.hpp"
#include "vector.hpp"
#include <vector>

namespace pf {
class Flock
{
 private:
  std::vector<Boid> flock_;

 public:
  Flock();
  Flock(std::vector<Boid>);

  Boid get_boids() const;

  

};
} // namespace pf

#endif