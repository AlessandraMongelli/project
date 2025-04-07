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

  float d;
  float ds = 0.5f;
  float s  = 0.7f;
  float a  = 0.7f;
  std::vector<Boid> entries_{};

 public:
  Boid();
  Boid(Vector, Vector);

  void add(Boid&);
  int size()
  {
    return entries_.size();
  }

  Vector get_position() const;
  Vector get_velocity() const;

  Vector Separation(const Vector&) const;
  Vector Alignment(const Vector&, const Vector&) const;
};
} // namespace pf

#endif