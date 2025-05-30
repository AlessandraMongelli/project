#ifndef FLOCK_H
#define FLOCK_H
#include "boid.hpp"

namespace pf {
struct Statistics
{
  float average_dist;
  float dev_dist;
  float average_vel;
  float dev_vel;
};

class Flock
{
 private:
  std::vector<Boid> flock_;
  std::vector<Boid> predators_;
  const float d_;
  const float ds_;
  const float s_;
  const float a_;
  const float c_;
  const float max_speed_;
  const float min_speed_;

 public:
  Flock(const float d, const float ds, const float s, const float a,
        const float c, const float max_vel, const float min_vel);

  void add_boids(const std::vector<Boid>&);
  const std::vector<Boid>& get_flock() const;
  const std::vector<Boid>& get_predators() const;
  size_t size();

  Vector flock_separation(const Boid&,
                          const std::vector<Boid>& neighbors) const;
  Vector flock_alignment(const Boid&,
                         const std::vector<Boid>& flock_neighbors) const;
  Vector flock_cohesion(const Boid&,
                        const std::vector<Boid>& flock_neighbors) const;
  Vector avoid_predators(const Boid&);
  Vector chase_prey(const Boid&, const std::vector<Boid>& flock_neighbors);

  void predators_update(float delta_t);
  void flock_update(float delta_t);
  Statistics flock_state() const;
};
} // namespace pf

#endif