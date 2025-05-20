#include "flock.hpp"
#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

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
  return flock_.size() + predators_.size();
};

Vector Flock::flock_separation(const Boid& boid,
                               const std::vector<Boid>& neighbors) const
{
  return boid.separation(neighbors, ds_, s_);
}

Vector Flock::flock_alignment(const Boid& boid,
                              const std::vector<Boid>& flock_neighbors) const
{
  if (boid.get_predator() == true) {
    return {0., 0.};
  }
  return boid.alignment(flock_neighbors, a_);
}

Vector Flock::flock_cohesion(const Boid& boid,
                             const std::vector<Boid>& flock_neighbors) const
{
  if (boid.get_predator() == true) {
    return {0., 0.};
  }
  return boid.cohesion(flock_neighbors, c_);
}

Vector Flock::avoid_predators(const Boid& boid)
{
  Vector steer(0., 0.);
  Vector position = boid.get_position();
  for (auto& predator : predators_) {
    if (predator.get_position().distance(position) < d_
        && predator.get_position().distance(position) > ds_) {
      steer += (position - predator.get_position())
             * (1.0f / predator.get_position().distance(position)) * 15.0f;
    } else if (predator.get_position().distance(position) < ds_
               && predator.get_position().distance(position) != 0) {
      steer += (position - predator.get_position())
             * (1.0f / predator.get_position().distance(position)) * 25.0f;
    }
  }
  return steer;
}

Vector Flock::chase_prey(const Boid& boid, const std::vector<Boid>& neighbors)
{
  Vector vp(0., 0.);
  Vector position = boid.get_position();

  auto compare_distances = [&position](const Boid& boid1, const Boid& boid2) {
    return boid1.get_position().distance(position)
         < boid2.get_position().distance(position);
  };

  auto closest =
      std::min_element(neighbors.begin(), neighbors.end(), compare_distances);

  if (closest != neighbors.end()
      && (closest->get_position() - position).norm() != 0) {
    vp += (closest->get_position() - position)
        * (1.0f / (closest->get_position() - position).norm()) * 10.0f;
  }
  return vp;
}

void Flock::predators_update(float delta_t)
{
  for (auto& boid : predators_) {
    std::vector<Boid> flock_neighbors     = boid.neighboring(flock_, d_);
    std::vector<Boid> predators_neighbors = boid.neighboring(predators_, d_);

    Vector delta_v = flock_separation(boid, predators_neighbors)
                   + chase_prey(boid, flock_neighbors);

    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed_, min_speed_);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 12.5f);
    boid.update_position(boid.get_velocity() * delta_t);
  }
}

void Flock::flock_update(float delta_t)
{
  for (auto& boid : flock_) {
    std::vector<Boid> flock_neighbors     = boid.neighboring(flock_, d_);
    std::vector<Boid> predators_neighbors = boid.neighboring(predators_, d_);

    Vector delta_v = flock_separation(boid, flock_neighbors)
                   + flock_separation(boid, predators_neighbors)
                   + flock_alignment(boid, flock_neighbors)
                   + flock_cohesion(boid, flock_neighbors)
                   + avoid_predators(boid);

    boid.update_velocity(delta_v);
    boid.speed_limit(max_speed_, min_speed_);
    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 15.0f);
    boid.update_position(boid.get_velocity() * delta_t);
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
        sum_dist2 += static_cast<float>(pow(dist, 2));
      }
    }

    float n = static_cast<float>(flock_.size() * (flock_.size() - 1)) / 2.0f;
    float average_dist  = sum_dist / n;
    float average_dist2 = sum_dist2 / n;
    float dev_dist      = static_cast<float>(sqrt(average_dist2 - pow(average_dist, 2)));

    float sum_vel = 0.0f, sum_vel2 = 0.0f;
    for (const auto& boid : flock_) {
      float speed = boid.get_velocity().norm();
      sum_vel += speed;
      sum_vel2 += static_cast<float>(pow(speed, 2));
    }

    float average_vel  = sum_vel / static_cast<float>(flock_.size());
    float average_vel2 = sum_vel2 / static_cast<float>(flock_.size());
    float dev_vel      = static_cast<float>(sqrt(average_vel2 - pow(average_vel, 2)));

    return {average_dist, dev_dist, average_vel, dev_vel};
  }
}
}; // namespace pf
