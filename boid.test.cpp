#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid Class")
{
  const pf::Vector xb1(0., 0.);
  const pf::Vector vb1(0., 0.);
  //const float va1(180.0f);

  const pf::Vector xb2(0.5, 0.5);
  const pf::Vector vb2(0.5, 0.5);
  //const float va2(180.0f);

  const pf::Vector xb3(1.0, 1.0);
  const pf::Vector vb3(1.0, 1.0);
  //const float va3(180.0f);

  pf::Boid boid1(xb1, vb1);
  pf::Boid boid2(xb2, vb2);
  pf::Boid boid3(xb3, vb3);

  std::vector<pf::Boid> boids      = {boid1, boid2, boid3};
  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, 1.0f);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, 1.0f);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, 1.0f);

  SUBCASE("Testing Neighboring function")
  {
    CHECK(boid2.neighboring(boids, 1.0f).size() == 1);
    CHECK(boid1.neighboring(boids, 1.0f).size() == 1);
  }

  SUBCASE("Testing the Separation rule")
  {
    CHECK(boid1.separation(neighbors1, 0.8f, 1.0f).get_x()
          == doctest::Approx(-0.5f));
    CHECK(boid1.separation(neighbors1, 0.8f, 1.0f).get_y()
          == doctest::Approx(-0.5f));
    CHECK(boid2.separation(neighbors2, 0.8f, 1.0f).get_x()
          == doctest::Approx(0.5));
    CHECK(boid2.separation(neighbors2, 0.8f, 1.0f).get_y()
          == doctest::Approx(0.5));
  }

  SUBCASE("Testing the Alignment rule")
  {
    CHECK(boid1.alignment(neighbors1, 0.7f).get_x() == doctest::Approx(0.35));
    CHECK(boid1.alignment(neighbors1, 0.7f).get_y() == doctest::Approx(0.35));
    CHECK(boid2.alignment(neighbors2, 0.7f).get_x() == doctest::Approx(0.0));
    CHECK(boid2.alignment(neighbors2, 0.7f).get_y() == doctest::Approx(0.0));
    CHECK(boid3.alignment(neighbors3, 0.7f).get_x() == doctest::Approx(-0.35));
    CHECK(boid3.alignment(neighbors3, 0.7f).get_y() == doctest::Approx(-0.35));
  }

  SUBCASE("Testing the Cohesion rule")
  {
    CHECK(boid1.cohesion(neighbors1, 0.4f).get_x() == doctest::Approx(0.2));
    CHECK(boid1.cohesion(neighbors1, 0.4f).get_y() == doctest::Approx(0.2));
    CHECK(boid2.cohesion(neighbors2, 0.4f).get_x() == doctest::Approx(0.0));
    CHECK(boid2.cohesion(neighbors2, 0.4f).get_y() == doctest::Approx(0.0));
    CHECK(boid3.cohesion(neighbors3, 0.4f).get_x() == doctest::Approx(-0.2));
    CHECK(boid3.cohesion(neighbors3, 0.4f).get_y() == doctest::Approx(-0.2));
  }

  SUBCASE("Testing get_...()")
  {
    CHECK(boid1.get_position().get_x() == doctest::Approx(0.0f));
    CHECK(boid1.get_position().get_y() == doctest::Approx(0.0f));
    CHECK(boid2.get_position().get_x() == doctest::Approx(0.5f));
    CHECK(boid2.get_position().get_y() == doctest::Approx(0.5f));
    CHECK(boid3.get_position().get_x() == doctest::Approx(1.0f));
    CHECK(boid3.get_position().get_y() == doctest::Approx(1.0f));
    CHECK(boid1.get_velocity().get_x() == doctest::Approx(0.0f));
    CHECK(boid1.get_velocity().get_y() == doctest::Approx(0.0f));
    CHECK(boid2.get_velocity().get_x() == doctest::Approx(0.5f));
    CHECK(boid2.get_velocity().get_y() == doctest::Approx(0.5f));
    CHECK(boid3.get_velocity().get_x() == doctest::Approx(1.0f));
    CHECK(boid3.get_velocity().get_y() == doctest::Approx(1.0f));
  }
}
