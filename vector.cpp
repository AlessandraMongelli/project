#include "vector.hpp"
#include <cassert>
#include <cmath>

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

Vector& Vector::operator+=(const Vector& w)
{
  x_ += w.x_;
  y_ += w.y_;
  return *this;
}

Vector Vector::operator+(const Vector& w) const
{
  Vector result = *this;
  return result += w;
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

float Vector::distance(const Vector& w) const
{
  const float dist = std::sqrt(std::pow(x_ - w.x_, 2) + std::pow(y_ - w.y_, 2));
  assert(dist >= 0.);
  return dist;
}

float Vector::norm() const
{
  const float norm = std::sqrt(std::pow(x_, 2) + std::pow(y_, 2));
  assert(norm >= 0.);
  return norm;
};

float Vector::product(const Vector& w) const
{
  float product = x_ * w.x_ + y_ * w.y_;
  return product;
};

bool Vector::operator==(const Vector& w) const
{
  return (w.x_ == x_ && w.y_ == y_);
};
} // namespace pf