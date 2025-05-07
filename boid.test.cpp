#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid Class")
{
  const pf::Vector xb1(0., 0.);
  const pf::Vector vb1(0., 0.);

  const pf::Vector xb2(0.5, 0.5);
  const pf::Vector vb2(0.5, 0.5);

  const pf::Vector xb3(1.0, 1.0);
  const pf::Vector vb3(1.0, 1.0);

  pf::Boid boid1(xb1, vb1, va1);
  pf::Boid boid2(xb2, vb2, va2);
  pf::Boid boid3(xb3, vb3, va3);

  std::vector<pf::Boid> boids      = {boid1, boid2, boid3};
  std::vector<pf::Boid> neighbors1 = boid1.Neighboring(boids, 1.);
  std::vector<pf::Boid> neighbors2 = boid2.Neighboring(boids, 1.);
  std::vector<pf::Boid> neighbors3 = boid3.Neighboring(boids, 1.);

  SUBCASE("Testing Neighboring function")
  {
    CHECK(boid2.Neighboring(boids, 1.).size() == 2);
    CHECK(boid1.Neighboring(boids, 1.).size() == 1);
  }

  SUBCASE("Testing the Separation rule")
  {
    CHECK(boid1.Separation(neighbors1, 0.8, 1.).get_x()
          == doctest::Approx(-0.5));
    CHECK(boid1.Separation(neighbors1, 0.8, 1.).get_y()
          == doctest::Approx(-0.5));
    CHECK(boid2.Separation(boid2.Neighboring(boids, 1.), 0.8, 1.).get_x()
          == doctest::Approx(0.5));
    CHECK(boid2.Separation(boid2.Neighboring(boids, 1.), 0.8, 1.).get_y()
          == doctest::Approx(0.5));
  }

  SUBCASE("Testing the Alignment rule")
  {
    CHECK(boid1.Alignment(neighbors1, 0.7).get_x() == doctest::Approx(0.35));
    CHECK(boid1.Alignment(neighbors1, 0.7).get_y() == doctest::Approx(0.35));
    CHECK(boid2.Alignment(neighbors2, 0.7).get_x() == doctest::Approx(0.0));
    CHECK(boid2.Alignment(neighbors2, 0.7).get_y() == doctest::Approx(0.0));
    CHECK(boid3.Alignment(neighbors3, 0.7).get_x() == doctest::Approx(-0.35));
    CHECK(boid3.Alignment(neighbors3, 0.7).get_y() == doctest::Approx(-0.35));
  }

  SUBCASE("Testing the Cohesion rule")
  {
    CHECK(boid1.Cohesion(neighbors1, 0.4).get_x() == doctest::Approx(0.2));
    CHECK(boid1.Cohesion(neighbors1, 0.4).get_y() == doctest::Approx(0.2));
    CHECK(boid2.Cohesion(neighbors2, 0.4).get_x() == doctest::Approx(0.0));
    CHECK(boid2.Cohesion(neighbors2, 0.4).get_y() == doctest::Approx(0.0));
    CHECK(boid3.Cohesion(neighbors3, 0.4).get_x() == doctest::Approx(-0.2));
    CHECK(boid3.Cohesion(neighbors3, 0.4).get_y() == doctest::Approx(-0.2));
  }

  SUBCASE("Testing the Flocking rule")
  {
    CHECK(boid1.Flocking(neighbors1).get_x() == doctest::Approx(0.05));
    CHECK(boid1.Flocking(neighbors1).get_y() == doctest::Approx(0.05));
    CHECK(boid2.Flocking(neighbors2).get_x() == doctest::Approx(0.5));
    CHECK(boid2.Flocking(neighbors2).get_y() == doctest::Approx(0.5));
  }

  SUBCASE("Testing get_...()")
  {
    CHECK(boid1.get_position().get_x() == doctest::Approx(0.));
    CHECK(boid1.get_position().get_y() == doctest::Approx(0.));
    CHECK(boid2.get_position().get_x() == doctest::Approx(0.5));
    CHECK(boid2.get_position().get_y() == doctest::Approx(0.5));
    CHECK(boid3.get_position().get_x() == doctest::Approx(1.));
    CHECK(boid3.get_position().get_y() == doctest::Approx(1.));
    CHECK(boid1.get_velocity().get_x() == doctest::Approx(0.));
    CHECK(boid1.get_velocity().get_y() == doctest::Approx(0.));
    CHECK(boid2.get_velocity().get_x() == doctest::Approx(0.5));
    CHECK(boid2.get_velocity().get_y() == doctest::Approx(0.5));
    CHECK(boid3.get_velocity().get_x() == doctest::Approx(1.));
    CHECK(boid3.get_velocity().get_y() == doctest::Approx(1.));
  }
}
