#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.) {};

Boid::Boid(Vector xb, Vector vb)
{
  x_b = xb;
  v_b = vb;
};

Vector Boid::get_position() const
{
  return x_b;
};

Vector Boid::get_velocity() const
{
  return v_b;
};

std::vector<Boid> Boid::Neighboring(const std::vector<Boid>& boids, float d)
{
  std::vector<Boid> neighbors;
  for (int i = 0; i < boids.size(); i++) {
    if (x_b == boids[i].get_position()) {
      continue;
    } else if (x_b.distance(boids[i].get_position()) < d) {
      neighbors.push_back(boids[i]);
    }
  }
  return neighbors;
}

Vector Boid::Separation(const std::vector<Boid>& neighbors, float ds,
                        float s) const
{
  Vector v_1(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    if (x_b == neighbors[i].get_position()
        || (x_b).distance(neighbors[i].get_position()) > ds) {
      continue;
    } else {
      v_1 += (x_b - neighbors[i].get_position()) * s;
    }
  }
  return v_1;
}

/*Vector Boid::Alignment(const Boid& boid2) const
{
  Vector v_2(0., 0.);
  if (boid2.get_position() == x_b && boid2.get_velocity() == v_b) {
  } else {
    v_2 += (boid2.get_velocity() * (1 / (n - 1))) * a;
  }
  return v_2 - v_b;
}

Vector Boid::Cohesion(const Boid& boid2) const
{
  Vector v_3(0., 0.);
  Vector cm = boid2.get_position() * (1 / (n - 1));
  if (boid2.get_position() == x_b && boid2.get_velocity() == v_b) {
  } else {
    v_3 += (cm - x_b) * c;
  }
  return v_3;
}*/
}; // namespace pf
