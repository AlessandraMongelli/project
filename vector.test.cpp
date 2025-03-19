#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"
#include "doctest.h"

TEST_CASE("Testing get_...()")
{
  pf::Vector v(3, 2);
  
  CHECK(v.get_x() == doctest::Approx(3.));
  CHECK(v.get_y() == doctest::Approx(2.));
}
