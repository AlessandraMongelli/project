#include "flock.hpp"
#include "boid.hpp"
#include "vector.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace pf {

Flock::Flock(const float d, const float ds, const float s, const float a,
             const float c, const float max_speed, const float min_speed)
    : d_(d)
    , ds_(ds)
    , s_(s)
    , a_(a)
    , c_(c)
    , max_speed_(max_speed)
    , min_speed_(min_speed) {};

void Flock::add_boids(const std::vector<Boid>& boids)
{
  for (auto& boid : boids) {
    flock_.push_back(boid);
  }
}

void Flock::add_predators(const std::vector<Boid>& predators)
{
  for (auto& predator : predators) {
    predators_.push_back(predator);
  }
}

std::vector<Boid> Flock::get_flock() const
{
  return flock_;
};

std::vector<Boid> Flock::get_predators() const
{
  return predators_;
}

size_t Flock::size()
{
  return flock_.size();
};

Vector Flock::flock_separation(const Boid& boid,
                               const std::vector<Boid>& neighbors) const
{
  return boid.separation(neighbors, ds_, s_);
}

Vector Flock::flock_alignment(const Boid& boid,
                              const std::vector<Boid>& neighbors) const
{
  return boid.alignment(neighbors, a_);
}

Vector Flock::flock_cohesion(const Boid& boid,
                             const std::vector<Boid>& neighbors) const
{
  return boid.cohesion(neighbors, c_);
}

Vector Flock::avoid_predators(const Boid& boid)
{
  Vector steer(0., 0.);
  Vector p = boid.get_position();
  for (auto& predator : predators_) {
    if (predator.get_position().distance(p) < d_)
      steer += (p - predator.get_position())
             * (1.0f / predator.get_position().distance(p));
  }

  return steer;
}

Vector Flock::chase_prey(const Boid& boid, const std::vector<Boid>& neighbors)
{
  Vector vp(0., 0.);
  Vector pos = boid.get_position();

  auto compare_distances = [&pos](const Boid& boid1, const Boid& boid2) {
    return boid1.get_position().distance(pos)
         < boid2.get_position().distance(pos);
  };

  auto closest =
      std::min_element(neighbors.begin(), neighbors.end(), compare_distances);

  if (closest != neighbors.end()) {
    vp += (closest->get_position() - pos)
        * (1.0f / (closest->get_position() - pos).norm());
  }
  return vp;
}

void Flock::predators_update()
{
  for (auto& boid : predators_) {
    std::vector<Boid> flock_neighbors = boid.neighboring(flock_, d_);

    Vector delta_v = chase_prey(boid, flock_neighbors);
    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed_, min_speed_);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 0.5f);
    // boid.edges_behavior(800.0f, 600.0f);
    boid.update_position(boid.get_velocity());
  }
}

void Flock::flock_update()
{
  // Avoid copies: create a vector of references to all boids
  std::vector<std::reference_wrapper<const Boid>> all_boids;
  all_boids.reserve(flock_.size() + predators_.size());

  for (const auto& b : flock_)
    all_boids.push_back(std::cref(b));
  for (const auto& p : predators_)
    all_boids.push_back(std::cref(p));

  for (auto& boid : flock_) {
    std::vector<Boid> neighbors; // Keep using real Boids for compatibility
    for (const Boid& other : all_boids) {
      if (boid.get_position() == other.get_position())
        continue;
      if (boid.get_position().distance(other.get_position()) < d_) {
        neighbors.push_back(other); // Still copying here, optional to refactor
      }
    }

    Vector delta_v = flock_separation(boid, neighbors)
                   + flock_alignment(boid, neighbors)
                   + flock_cohesion(boid, neighbors) + avoid_predators(boid);

    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed_, min_speed_);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 0.5f);
    // boid.edges_behavior(800.0f, 600.0f);
    boid.update_position(boid.get_velocity());
  }
}

Statistics Flock::flock_state() const
{
  if (flock_.size() < 2) {
    return {0., 0., 0., 0.};
  } else {
    float sum_dist = 0.0f, sum_dist2 = 0.0f;
    for (size_t i = 0; i < flock_.size(); ++i) {
      for (size_t j = i + 1; j < flock_.size(); ++j) {
        float dist =
            flock_[i].get_position().distance(flock_[j].get_position());
        sum_dist += dist;
        sum_dist2 += std::pow(dist, 2);
      }
    }

    float n = static_cast<float>(flock_.size() * (flock_.size() - 1)) / 2.0f;
    float average_dist  = sum_dist / n;
    float average_dist2 = sum_dist2 / n;
    float dev_dist      = std::sqrt(average_dist2 - std::pow(average_dist, 2));

    float sum_vel = 0.0f, sum_vel2 = 0.0f;
    for (const auto& boid : flock_) {
      float speed = boid.get_velocity().norm();
      sum_vel += speed;
      sum_vel2 += std::pow(speed, 2);
    }

    float average_vel  = sum_vel / flock_.size();
    float average_vel2 = sum_vel2 / flock_.size();
    float dev_vel      = std::sqrt(average_vel2 - std::pow(average_vel, 2));

    return {average_dist, dev_dist, average_vel, dev_vel};
  }
}

}; // namespace pf
