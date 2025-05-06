#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.)
    , view_angle(0.) {};

Boid::Boid(Vector xb, Vector vb, float va)
{
  x_b = xb;
  v_b = vb;
  view_angle = va;
};

Vector Boid::get_position() const
{
  return x_b;
};

Vector Boid::get_velocity() const
{
  return v_b;
};

float Boid::get_view_angle() const{
return view_angle;
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
  return this->get_velocity() + v_1;
}

Vector Boid::Alignment(const std::vector<Boid>& neighbors, float a) const
{
  Vector v_2(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    if (x_b == neighbors[i].get_position()) {
      continue;
    } else {
      v_2 += neighbors[i].get_velocity() * (1.0f / neighbors.size());
    }
  }
  return (v_2 - this->get_velocity()) * a;
}

Vector Boid::Cohesion(const std::vector<Boid>& neighbors, float c) const
{
  Vector v_3(0., 0.);
  Vector x_c(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    if (x_b == neighbors[i].get_position()) {
      continue;
    } else {
      x_c += neighbors[i].get_position() * (1.0f / neighbors.size());
    }
  }
  return v_3 = (x_c - this->get_position()) * c;
}

Vector Boid::Flocking(const std::vector<Boid>& neighbors) const
{
  Vector v_f(0., 0.);
  Vector v_1 = Separation(neighbors, 0.8, 1.0);
  Vector v_2 = Alignment(neighbors, 0.7);
  Vector v_3 = Cohesion(neighbors, 0.4);
  return v_f = v_1 + v_2 + v_3;
}

/*Vector Boid::Alignment(const Boid& boid2) const
{
  Vector v_2(0., 0.);
  for (int i = 0; i<neighbors.size(); i++)
  if (boid2.get_position() == x_b && boid2.get_velocity() == v_b) {
  } else {
    v_2 += (boid2.get_velocity() * (1 / (n - 1))) * a;
  }
  return this->get.velocity() - v_b;
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
