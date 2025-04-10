#include "boid.hpp"
#include "vector.hpp"
#include <cmath>

namespace pf {

Boid::Boid()
    : x_b(0., 0.)
    , v_b(0., 0.) {};

Boid::Boid(Vector xb, Vector vb)
{
  x_b = xb;
  v_b = vb;
};

void Boid::add(Boid& boid)
{
  entries_.push_back(boid);
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

Vector Boid::Alignment(const Boid& boid) const
{
  Vector v_2(0., 0.);
  Vector sum(0., 0.);
  int n = entries_.size();
  for (int i = 0; i < n; i++) {
    sum += entries_[i].get_velocity();
  }

  v_2 == ((sum * (1 / (n - 1))) - v_b) * a;

  return v_2;
};

} // namespace pf
