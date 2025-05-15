#include "flock.hpp"
#include "boid.hpp"
#include "vector.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace pf {

Flock::Flock()
    : flock_()
    , predators_() {};

Flock::Flock(std::vector<Boid> flock, std::vector<Boid> predators)
{
  flock_     = flock;
  predators_ = predators;
}

void Flock::add_boids(const std::vector<Boid>& boids)
{
  for (auto& boid : boids) {
    if (boid.get_predator() == true) {
      predators_.push_back(boid);
    } else {
      flock_.push_back(boid);
    }
  }
}

const std::vector<Boid>& Flock::get_flock() const
{
  return flock_;
};

const std::vector<Boid>& Flock::get_predators() const
{
  return predators_;
}

size_t Flock::size()
{
  return flock_.size();
};

Vector Flock::flock_separation(const Boid& boid,
                               const std::vector<Boid>& neighbors, float ds,
                               float s) const
{
  return boid.separation(neighbors, ds, s);
}

Vector Flock::flock_alignment(const Boid& boid,
                              const std::vector<Boid>& neighbors, float a) const
{
  if (boid.get_predator() == true) {
    return {0., 0.};
  }

  return boid.alignment(neighbors, a);
}

Vector Flock::flock_cohesion(const Boid& boid,
                             const std::vector<Boid>& neighbors, float c) const
{
  if (boid.get_predator() == true) {
    return {0., 0.};
  }

  return boid.cohesion(neighbors, c);
}

Vector Flock::avoid_predators(const Boid& boid, float ap)
{
  if (boid.get_predator() == false) {
    Vector steer(0., 0.);
    Vector p = boid.get_position();
    for (auto& predator : predators_) {
      if (predator.get_position().distance(p) < ap)
        steer += (p - predator.get_position())
               * (1.0f / predator.get_position().distance(p));
    }
    return steer;
  } else {
    return {0., 0.};
  }
}

Vector Flock::chase_prey(const Boid& boid, const std::vector<Boid>& neighbors,
                         float cp)
{
  if (boid.get_predator() == true) {
    Vector vp(0., 0.);
    Vector p = boid.get_position();

    auto compare_distances = [&p](const Boid& boid1, const Boid& boid2) {
      return boid1.get_position().distance(p)
           < boid2.get_position().distance(p);
    };

    auto closest =
        std::min_element(neighbors.begin(), neighbors.end(), compare_distances);

    if (closest != neighbors.end()) {
      vp += (closest->get_position() - p)
          * (1.0f / (closest->get_position() - p).norm()) * cp;
    }
    return vp;
  } else {
    return {0., 0.};
  }
}

void Flock::predators_update(float d, float max_speed, float min_speed,
                             float cp)
{
  for (auto& boid : predators_) {
    std::vector<Boid> flock_neighbors = boid.neighboring(flock_, d);

    Vector delta_v = chase_prey(boid, flock_neighbors, cp);
    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed, min_speed);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 0.5f);
    //boid.edges_behavior(800.0f, 600.0f);
    boid.update_position(boid.get_velocity());
  }
}

void Flock::flock_update(float d, float ds, float s, float a, float c,
                         float max_speed, float min_speed, float ap)
{
  std::vector<Boid> boids;
  boids.reserve(flock_.size() + predators_.size());
  boids.insert(boids.end(), flock_.begin(), flock_.end());
  boids.insert(boids.end(), predators_.begin(), predators_.end());

  for (auto& boid : flock_) {
    std::vector<Boid> flock_neighbors = boid.neighboring(boids, d);

    Vector delta_v = flock_separation(boid, flock_neighbors, ds, s)
                   + flock_alignment(boid, flock_neighbors, a)
                   + flock_cohesion(boid, flock_neighbors, c)
                   + avoid_predators(boid, ap);

    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed, min_speed);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 0.5f);
    // boid.edges_behavior(800.0f, 600.0f);
    boid.update_position(boid.get_velocity());
  }
}
}; // namespace pf
