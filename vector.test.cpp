#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"
#include "doctest.h"

TEST_CASE("Testing get_...()")
{
  pf::Vector v(3.12f, 6.99f);
  
  CHECK(v.get_x() == doctest::Approx(3.12f));
  CHECK(v.get_y() == doctest::Approx(6.99f));
}

TEST_CASE("Testing set_...()")
{
  pf::Vector v(3.12f, 6.99f);

  CHECK(v.set_x(6.11f) == doctest::Approx(6.11f));
  CHECK(v.set_y(0.81f) == doctest::Approx(0.81f));
}

