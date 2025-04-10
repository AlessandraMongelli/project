#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid Class")
{
  pf::Boid boids;
  const pf::Vector xb1(0., 0.);
  const pf::Vector vb1(0., 0.);

  const pf::Vector xb2(0.5, 0.5);
  const pf::Vector vb2(0.5, 0.5);

  const pf::Vector xb3(-1.0, -1.0);
  const pf::Vector vb3(1.0, 1.0);

  pf::Boid boid1(xb1, vb1);
  pf::Boid boid2(xb2, vb2);
  pf::Boid boid3(xb3, vb3);

  SUBCASE("Testing get_...()")
  {
    CHECK(boid1.get_position().get_x() == doctest::Approx(0.));
    CHECK(boid1.get_position().get_y() == doctest::Approx(0.));
    CHECK(boid2.get_position().get_x() == doctest::Approx(0.5));
    CHECK(boid2.get_position().get_y() == doctest::Approx(0.5));
    CHECK(boid3.get_position().get_x() == doctest::Approx(-1.));
    CHECK(boid3.get_position().get_y() == doctest::Approx(-1.));
    CHECK(boid1.get_velocity().get_x() == doctest::Approx(0.));
    CHECK(boid1.get_velocity().get_y() == doctest::Approx(0.));
    CHECK(boid2.get_velocity().get_x() == doctest::Approx(0.5));
    CHECK(boid2.get_velocity().get_y() == doctest::Approx(0.5));
    CHECK(boid3.get_velocity().get_x() == doctest::Approx(1.));
    CHECK(boid3.get_velocity().get_y() == doctest::Approx(1.));
  }

  SUBCASE("Testing the Separation rule")
  {
    CHECK(boid1.Separation(xb2).get_x() == doctest::Approx(-0.35));
    CHECK(boid1.Separation(xb2).get_y() == doctest::Approx(-0.35));
    CHECK(boid2.Separation(xb1).get_x() == doctest::Approx(0.35));
    CHECK(boid2.Separation(xb1).get_y() == doctest::Approx(0.35));
    CHECK(boid1.Separation(xb3).get_x() == doctest::Approx(0.7));
    CHECK(boid1.Separation(xb3).get_y() == doctest::Approx(0.7));
    CHECK(boid1.Separation(xb1).get_x() == doctest::Approx(0.));
    CHECK(boid1.Separation(xb1).get_y() == doctest::Approx(0.));
  }

  SUBCASE("Testing size")
  {
    boids.add(boid1);
    boids.add(boid2);
    boids.add(boid3);
    CHECK(boid1.size() == 3);
  }
}
