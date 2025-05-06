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
  Boid(Vector, Vector);

  Vector get_position() const;
  Vector get_velocity() const;

  std::vector<Boid> Neighboring(const std::vector<Boid>&, float d);
  Vector Separation(const std::vector<Boid>&, float ds, float s) const;

  Vector Alignment(const std::vector<Boid>&, float a) const;
  /* Vector Alignment(const Boid&) const;
  Vector Cohesion(const Boid&) const; */
};
} // namespace pf

#endif