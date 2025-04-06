#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

namespace pf {

Boid::Boid()
    : x_b(0.f, 0.f)
    , v_b(0.f, 0.f) {};

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

Vector Boid::Separation(const Vector& xb2) const
{
  Vector v_1(0.,0.);
  if (x_b == xb2 || (x_b).distance(xb2) < ds) {
    return v_1;
  } else {
    v_1 += (xb2 - x_b) * (-s);
    return v_1;
  }
}

} // namespace pf
