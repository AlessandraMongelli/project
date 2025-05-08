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

  std::vector<Boid> Neighboring(const std::vector<Boid>&, float d);
  Vector Separation(const std::vector<Boid>&, float ds, float s) const;
  Vector Alignment(const std::vector<Boid>&, float a) const;
  Vector Cohesion(const std::vector<Boid>&, float c) const;
  
  void speed_limit(float max_speed);
  void update_velocity(const Vector&);
  void update_position(const Vector&);
};
} // namespace pf

#endif