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
  bool is_predator;
  mutable sf::ConvexShape shape;

 public:
  Boid();
  Boid(Vector, Vector, bool);
  
  void set_predator(bool);
  
  Vector get_position() const;
  Vector get_velocity() const;
  bool get_predator() const;

  bool operator==(const Boid&) const;

  std::vector<Boid> neighboring(const std::vector<Boid>&, float d) const;
  Vector separation(const std::vector<Boid>& neighbors, float ds,
                    float s) const;
  Vector alignment(const std::vector<Boid>& neighbors, float a) const;
  Vector cohesion(const std::vector<Boid>& neighbors, float c) const;

  void speed_limit(float max_speed, float min_speed);
  void update_velocity(const Vector&);
  void update_position(const Vector&);
  float rotate_angle() const;
  void edges_behavior(const float leftmargin, const float rightmargin,
                      const float topmargin, const float bottommargin,
                      const float t);
  // void edges_behavior(const float width, const float height);

  void draw(sf::RenderWindow& window) const;
};
} // namespace pf

#endif