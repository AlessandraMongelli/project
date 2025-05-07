#include "flock.hpp"
#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

namespace pf {

Flock::Flock()
    : flock_() {};

Flock::Flock(std::vector<Boid> flock)
{
  flock_ = flock;
};

Flock Flock::add_boids(const std::vector<Boid>& boids)
{
  for (int i = 0; i < boids.size(); i++) {
    flock_.push_back(boids[i]);
  }
}

Boid Flock::get_boids() const
{
  for (int i = 0; i < flock_.size(); i++) {
    return flock_[i];
  }
};

Boid Flock::Flock_Separation(const Boid& flockboids) const {};
Boid Flock::Flock_Alignment(const Boid& flockboids) const {};
Boid Flock::Flock_Cohesion(const Boid& flockboids) const {};
   
};


} // namespace pf
