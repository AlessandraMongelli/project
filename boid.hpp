#ifndef BOID_H
#define BOID_H
#include "vector.hpp"
#include <vector>

namespace pf {

class Boid
{
 private:
  Vector x_b;
  Vector v_b;
  float view_angle;

 public:
  Boid();
  Boid(Vector, Vector, float);

  Vector get_position() const;
  Vector get_velocity() const;
  float get_view_angle() const;
  float get_others_angle(const Boid&) const;

  std::vector<Boid> neighboring(const std::vector<Boid>&, float d);
  Vector separation(const std::vector<Boid>&, float ds, float s) const;
  Vector alignment(const std::vector<Boid>&, float a) const;
  Vector cohesion(const std::vector<Boid>&, float c) const;

  void speed_limit(float max_speed, float min_speed);
  void update_velocity(const Vector&);
  void update_position(const Vector&);
  float rotate_angle() const;
  void edges_behavior(const float max_width, const float max_height);
};
} // namespace pf

#endif