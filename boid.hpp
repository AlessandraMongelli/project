#ifndef BOID_H
#define BOID_H
#include "vector.hpp"

namespace pf {

class Boid
{
 private:
  Vector x_b;
  Vector v_b;

  float d;
  float ds = 0.5f;
  float s  = 0.7f;

 public:
  Boid();
  Boid(Vector, Vector);

  Vector get_position() const;
  Vector get_velocity() const;

  Vector Separation(const Vector&) const;
};
} // namespace pf

#endif