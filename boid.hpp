#ifndef BOID_H
#define BOID_H
#include "vector.hpp"
#include <SFML/Graphics.hpp>
#include <cassert>
#include <cstdlib>
#include <vector>

namespace pf {

class Boid
{
 private:
  Vector x_b;
  Vector v_b;
  // float view_angle;
  sf::ConvexShape shape;

 public:
  Boid();
  Boid(Vector, Vector); //, float);
  sf::ConvexShape set_shape();

  Vector get_position() const;
  Vector get_velocity() const;
  // float get_view_angle() const;

  std::vector<Boid> neighboring(const std::vector<Boid>&, float d);
  Vector separation(const std::vector<Boid>&, float ds, float s) const;
  Vector alignment(const std::vector<Boid>&, float a) const;
  Vector cohesion(const std::vector<Boid>&, float c) const;

  void speed_limit(float max_speed, float min_speed);
  void update_velocity(const Vector&);
  void update_position(const Vector&);
  // float other_angle(const Boid&) const;
  float rotate_angle() const;
  void edges_behavior(const float leftmargin, const float rightmargin,
                      const float topmargin, const float bottommargin,
                      const float t);
  void draw(sf::RenderWindow& window);
};
} // namespace pf

#endif