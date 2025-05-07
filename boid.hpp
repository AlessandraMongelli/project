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

<<<<<<< HEAD
  Vector update_velocity(const std::vector<Boid>&) const;

  Vector max_v(const Boid&) const;
=======
>>>>>>> 35fad241945319fc88b52f0e186be2df6bfd400e
};
} // namespace pf

#endif