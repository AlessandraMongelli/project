#ifndef VECTOR_H
#define VECTOR_H

namespace pf {
class Vector
{
 private:
  float x_;
  float y_;

 public:
  Vector();
  Vector(float x, float y);

  float get_x() const;
  float get_y() const;
  float set_x(float new_x);
  float set_y(float new_y);
  Vector operator+(const Vector&) const;
  Vector& operator+=(const Vector&);
  Vector operator-(const Vector&) const;
  Vector operator*(float scalar) const;
  bool operator==(const Vector&) const;
  float product(const Vector&) const;
  float distance(const Vector&) const;
  float norm() const;
};
} // namespace pf

#endif
