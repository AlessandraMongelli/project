#include "boid.hpp"
#include "vector.hpp"
#include <cmath>
#include <vector>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.) {};

Boid::Boid(Vector xb, Vector vb)
{
  x_b = xb;
  v_b = vb;
};

void Boid::add(Vector& x)
{
  entries_.push_back(x);
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
  Vector v_1(0., 0.);
  if ((x_b).distance(xb2) < ds) {
    v_1 += (xb2 - x_b) * (-s);
    return v_1;
  } else {
    return v_1;
  }
}

Vector Boid::Alignment(const Vector& vb2, const Vector& xb2) const
{
  Vector v_2(0., 0.);
  int n = entries_.size();
  v_2 == ((vb2 * (1 / (n - 1))) - v_b) * a;
  return v_2;
};

} // namespace pf
