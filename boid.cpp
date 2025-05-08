#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <cmath>
#include <numeric>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.)
    , view_angle(0.) {};

Boid::Boid(Vector xb, Vector vb, float va)
{
  x_b        = xb;
  v_b        = vb;
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

float Boid::get_view_angle() const
{
  return view_angle;
};

float Boid::get_other_angle(const Boid& boid) const
{
  float relative_x = std::abs(x_b.get_x() - boid.get_position().get_x());
  float relative_y = std::abs(x_b.get_y() - boid.get_position().get_y());

  return std::atan(relative_y / relative_x);
};

std::vector<Boid> Boid::neighboring(const std::vector<Boid>& boids, float d)
{
  std::vector<Boid> neighbors;
  for (int i = 0; i < boids.size(); i++) {
    if (x_b == boids[i].get_position()) {
      continue;
    } else if (x_b.distance(boids[i].get_position()) < d
               && get_other_angle(boids[i]) < view_angle) {
      neighbors.push_back(boids[i]);
    }
  }
  return neighbors;
}

Vector Boid::separation(const std::vector<Boid>& neighbors, float ds,
                        float s) const
{
  Vector v_1(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    if ((x_b).distance(neighbors[i].get_position()) > ds) {
      continue;
    } else {
      v_1 += (x_b - neighbors[i].get_position()) * s;
    }
  }

  return v_1;
}

Vector Boid::alignment(const std::vector<Boid>& neighbors, float a) const
{
  Vector v_2(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    v_2 += neighbors[i].get_velocity() * (1.0f / neighbors.size());
  }

  return (v_2 - this->get_velocity()) * a;
}

Vector Boid::cohesion(const std::vector<Boid>& neighbors, float c) const
{
  Vector x_c(0., 0.);
  for (int i = 0; i < neighbors.size(); i++) {
    x_c += neighbors[i].get_position() * (1.0f / neighbors.size());
  }

  return (x_c - this->get_position()) * c;
}

void Boid::speed_limit(float max_speed)
{
  float speed = this->get_velocity().norm();
  if (speed > max_speed) {
    v_b = v_b * (max_speed / speed);
  };
}

void Boid::update_position(const Vector& delta_x)
{
  x_b += delta_x;
}

void Boid::update_velocity(const Vector& delta_v)
{
  v_b += delta_v;
};

void Boid::edges_behavior(const float width, const float height)
{
  if (x_b.get_x() < 0) {
    x_b.set_x(width);
  }

  if (x_b.get_x() > width) {
    x_b.set_x(0);
  }

  if (x_b.get_y() < 0) {
    x_b.set_y(height);
  }

  if (x_b.get_y() > height) {
    x_b.set_y(0);
  }
};

}; // namespace pf
