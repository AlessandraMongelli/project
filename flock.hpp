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

  Flock add_boids(const std::vector<Boid>&);
  Boid get_boids() const;

  Boid Flock_Separation(const Boid&) const;

};
} // namespace pf

#endif