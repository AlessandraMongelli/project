#include "vector.hpp"
#include <cassert>
#include <cmath>

namespace pf {
Vector::Vector()
    : x_{0.0f}
    , y_{0.0f} {};

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

void Vector::set_x(float new_x)
{
  x_ = new_x;
};

void Vector::set_y(float new_y)
{
  y_ = new_y;
};

Vector Vector::operator+(const Vector& w) const
{
  Vector result = *this;
  return result += w;
}

Vector& Vector::operator+=(const Vector& w)
{
  x_ += w.x_;
  y_ += w.y_;
  return *this;
}

Vector Vector::operator-(const Vector& w) const
{
  Vector neg_w{-w.x_, -w.y_};
  return *this + neg_w;
}

Vector Vector::operator*(float c) const
{
  Vector prod{x_ * c, y_ * c};
  return prod;
}

bool Vector::operator==(const Vector& w) const
{
  return (w.x_ == x_ && w.y_ == y_);
};

float Vector::product(const Vector& w) const
{
  float product = x_ * w.x_ + y_ * w.y_;
  return product;
};

float Vector::distance(const Vector& w) const
{
  const float dist =
      static_cast<float>(sqrt(pow(x_ - w.x_, 2) + pow(y_ - w.y_, 2)));
  assert(dist >= 0.);
  return dist;
}

float Vector::norm() const
{
  const float norm = static_cast<float>(sqrt(pow(x_, 2) + pow(y_, 2)));
  assert(norm >= 0.);
  return norm;
};
} // namespace pf