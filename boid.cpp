#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <cmath>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.)
    , is_predator(0) {};

Boid::Boid(Vector xb, Vector vb, bool predator)
{
  x_b = xb;
  v_b = vb;
  is_predator = predator;

  {
    shape.setPointCount(3);

    shape.setFillColor(sf::Color::White);
    shape.setPoint(0, sf::Vector2f(0, -10));
    shape.setPoint(1, sf::Vector2f(-5, 5));
    shape.setPoint(2, sf::Vector2f(5, 5));

    shape.setFillColor(is_predator ? sf::Color::Red : sf::Color::White);
  };
}

void Boid::set_predator(bool value)
{
  is_predator = value;
}


Vector Boid::get_position() const
{
  return x_b;
};

Vector Boid::get_velocity() const
{
  return v_b;
};

bool Boid::get_predator() const {
 return is_predator;
}

bool Boid::operator==(const Boid& boid) const
{
  return x_b == boid.get_position();
}

std::vector<Boid> Boid::neighboring(const std::vector<Boid>& boids,
                                    float d) const
{
  std::vector<Boid> neighbors;
  for (auto& boid : boids) {
    if (x_b == boid.get_position()) {
      continue;
    } else if (x_b.distance(boid.get_position()) < d) {
      neighbors.push_back(boid);
    }
  }
  return neighbors;
}

Vector Boid::separation(const std::vector<Boid>& neighbors, float ds,
                        float s) const
{
  Vector v_1(0., 0.);
  for (auto& boid : neighbors) {
    if ((x_b).distance(boid.get_position()) > ds) {
      continue;
    } else {
      v_1 += (x_b - boid.get_position()) * s;
    }
  }

  return v_1;
}

Vector Boid::alignment(const std::vector<Boid>& neighbors, float a) const
{
  Vector v_2(0., 0.);
  for (auto& boid : neighbors) {
    v_2 += boid.get_velocity() * (1.0f / static_cast<float>(neighbors.size()));
  }

  return (v_2 - this->get_velocity()) * a;
}

Vector Boid::cohesion(const std::vector<Boid>& neighbors, float c) const
{
  Vector x_c(0., 0.);
  for (auto& boid : neighbors) {
    x_c += boid.get_position() * (1.0f / static_cast<float>(neighbors.size()));
  }

  return (x_c - this->get_position()) * c;
}

void Boid::speed_limit(float max_speed, float min_speed)
{
  float speed = this->get_velocity().norm();
  if (speed > max_speed) {
    v_b = v_b * (max_speed / speed);
  };
  if (speed < min_speed) {
    v_b = v_b * (min_speed / speed);
  }
}

void Boid::update_position(const Vector& delta_x)
{
  x_b += delta_x;
}

void Boid::update_velocity(const Vector& delta_v)
{
  v_b += delta_v;
};

float Boid::rotate_angle() const
{
  const float angle =
      static_cast<float>(atan2(v_b.get_y(), v_b.get_x()) * 180.0f / M_PI);
  return angle + 90.0f;
};

void Boid::edges_behavior(const float leftmargin, const float rightmargin,
                          const float topmargin, const float bottommargin,
                          const float t)
{
  if (x_b.get_x() < leftmargin) {
    v_b.set_x(v_b.get_x() + t);
  }

  if (x_b.get_x() > rightmargin) {
    v_b.set_x(v_b.get_x() - t);
  }

  if (x_b.get_y() > topmargin) {
    v_b.set_y(v_b.get_y() - t);
  }

  if (x_b.get_y() < bottommargin) {
    v_b.set_y(v_b.get_y() + t);
  }
};

/* void Boid::edges_behavior(const float width, const float height)
{
  if (x_b.get_x() < 0) {
    x_b.set_x(width);
  }

  if (x_b.get_x() > width) {
    x_b.set_x(0);
  }

  if (x_b.get_y() > height) {
    x_b.set_y(0);
  }

  if (x_b.get_y() < 0) {
    x_b.set_y(height);
  }
}; */

void Boid::draw(sf::RenderWindow& window) const
{
  shape.setPosition(x_b.get_x(), x_b.get_y());

  shape.setRotation(rotate_angle());

  window.draw(shape);
}

}; // namespace pf
