#ifndef FLOCK_H
#define FLOCK_H
#include "boid.hpp"
#include "vector.hpp"
#include <algorithm>
#include <numeric>
#include <vector>

namespace pf {
class Flock
{
 private:
  std::vector<Boid> flock_;
  std::vector<Boid> predators_;

 public:
  Flock();
  Flock(std::vector<Boid>, std::vector<Boid>);

  void add_boids(const std::vector<Boid>&);
  const std::vector<Boid>& get_flock() const;
  const std::vector<Boid>& get_predators() const;
  size_t size();

  Vector flock_separation(const Boid&, const std::vector<Boid>& neighbors,
                          float ds, float s) const;
  Vector flock_alignment(const Boid&, const std::vector<Boid>& neighbors,
                         float a) const;
  Vector flock_cohesion(const Boid&, const std::vector<Boid>& neighbors,
                        float c) const;
  Vector avoid_predators(const Boid&, float ap);
  Vector chase_prey(const Boid&, const std::vector<Boid>& neighbors, float cp);

  void predators_update(float d, float max_speed, float min_speed, float cp);
  void flock_update(float d, float ds, float s, float a, float c,
                    float max_speed, float min_speed, float ap);
};
} // namespace pf

#endif