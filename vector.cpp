#include "vector.hpp"

namespace pf {
Vector::Vector(): x_{0.0f}, y_{0.0f} {};
Vector::Vector(float x, float y)
{
  x_ = x;
  y_ = y;
};

float Vector::get_x() const
{
  return x_;
};
float Vector::get_y() const
{
  return y_;
};

float Vector::set_x(float new_x)
{
  return x_ = new_x;
};
float Vector::set_y(float new_y)
{
  return y_ = new_y;
};



} // namespace pf