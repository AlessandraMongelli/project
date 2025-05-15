#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Boid Class")
{
  const pf::Vector xb1(0., 0.);
  const pf::Vector vb1(5., -1.);
  // const float va1(180.0f);

  const pf::Vector xb2(1.5, 0.5);
  const pf::Vector vb2(-0.5, 2.5);
  // const float va2(180.0f);

  const pf::Vector xb3(2.2, -1.8);
  const pf::Vector vb3(3.0, 1.0);
  // const float va3(180.0f);

  const pf::Vector xb4(-1.0, -0.5);
  const pf::Vector vb4(-0.25, -2.0);

  pf::Boid boid1(xb1, vb1, 0);
  pf::Boid boid2(xb2, vb2, 0);
  pf::Boid boid3(xb3, vb3, 0);
  pf::Boid boid4(xb4, vb4, 0);

  std::vector<pf::Boid> boids      = {boid1, boid2, boid3, boid4};
  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, 2.5f);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, 2.0f);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, 3.5f);
  std::vector<pf::Boid> neighbors4 = boid4.neighboring(boids, 3.0f);

  SUBCASE("Testing Neighboring function")
  {
    CHECK(boid4.neighboring(boids, 3.0f).size() == 2);
    CHECK(boid3.neighboring(boids, 3.5f).size() == 3);
    CHECK(boid2.neighboring(boids, 2.0f).size() == 1);
    CHECK(boid1.neighboring(boids, 2.5f).size() == 2);
  }

  SUBCASE("Testing the Separation rule")
  {
    CHECK(boid1.separation(neighbors1, 2.5f, 1.0f).get_x()
          == doctest::Approx(-0.5f).epsilon(0.1));
    CHECK(boid1.separation(neighbors1, 2.5f, 1.0f).get_y()
          == doctest::Approx(0.0f).epsilon(0.1));
    CHECK(boid2.separation(neighbors2, 2.0f, 1.0f).get_x()
          == doctest::Approx(1.5).epsilon(0.1));
    CHECK(boid2.separation(neighbors2, 2.0f, 1.0f).get_y()
          == doctest::Approx(0.5).epsilon(0.1));
    CHECK(boid3.separation(neighbors3, 3.5f, 1.0f).get_x()
          == doctest::Approx(6.1).epsilon(0.1));
    CHECK(boid3.separation(neighbors3, 3.5f, 1.0f).get_y()
          == doctest::Approx(-5.4).epsilon(0.1));
    CHECK(boid4.separation(neighbors4, 3.0f, 1.0f).get_x()
          == doctest::Approx(-3.5).epsilon(0.1));
    CHECK(boid4.separation(neighbors4, 3.0f, 1.0f).get_y()
          == doctest::Approx(-1.5).epsilon(0.1));
  }

  SUBCASE("Testing the Alignment rule")
  {
    CHECK(boid1.alignment(neighbors1, 0.7f).get_x()
          == doctest::Approx(-3.763).epsilon(0.001));
    CHECK(boid1.alignment(neighbors1, 0.7f).get_y()
          == doctest::Approx(0.875).epsilon(0.001));
    CHECK(boid2.alignment(neighbors2, 0.7f).get_x()
          == doctest::Approx(3.85).epsilon(0.001));
    CHECK(boid2.alignment(neighbors2, 0.7f).get_y()
          == doctest::Approx(-2.45).epsilon(0.001));
    CHECK(boid3.alignment(neighbors3, 0.7f).get_x()
          == doctest::Approx(-1.108).epsilon(0.001));
    CHECK(boid3.alignment(neighbors3, 0.7f).get_y()
          == doctest::Approx(-0.817).epsilon(0.001));
    CHECK(boid4.alignment(neighbors4, 0.7f).get_x()
          == doctest::Approx(1.75).epsilon(0.001));
    CHECK(boid4.alignment(neighbors4, 0.7f).get_y()
          == doctest::Approx(1.925).epsilon(0.001));
  }

  SUBCASE("Testing the Cohesion rule")
  {
    CHECK(boid1.cohesion(neighbors1, 0.4f).get_x()
          == doctest::Approx(0.1).epsilon(0.001));
    CHECK(boid1.cohesion(neighbors1, 0.4f).get_y()
          == doctest::Approx(0.).epsilon(0.001));
    CHECK(boid2.cohesion(neighbors2, 0.4f).get_x()
          == doctest::Approx(-0.6).epsilon(0.001));
    CHECK(boid2.cohesion(neighbors2, 0.4f).get_y()
          == doctest::Approx(-0.2).epsilon(0.001));
    CHECK(boid3.cohesion(neighbors3, 0.4f).get_x()
          == doctest::Approx(-0.813).epsilon(0.001));
    CHECK(boid3.cohesion(neighbors3, 0.4f).get_y()
          == doctest::Approx(0.72).epsilon(0.001));
    CHECK(boid4.cohesion(neighbors4, 0.4f).get_x()
          == doctest::Approx(0.7).epsilon(0.001));
    CHECK(boid4.cohesion(neighbors4, 0.4f).get_y()
          == doctest::Approx(0.3).epsilon(0.001));
  }

  SUBCASE("Testing get_...()")
  {
    CHECK(boid1.get_position().get_x() == doctest::Approx(0.0f));
    CHECK(boid1.get_position().get_y() == doctest::Approx(0.0f));
    CHECK(boid2.get_position().get_x() == doctest::Approx(1.5f));
    CHECK(boid2.get_position().get_y() == doctest::Approx(0.5f));
    CHECK(boid3.get_position().get_x() == doctest::Approx(2.2f));
    CHECK(boid3.get_position().get_y() == doctest::Approx(-1.8f));
    CHECK(boid4.get_position().get_x() == doctest::Approx(-1.0f));
    CHECK(boid4.get_position().get_y() == doctest::Approx(-0.5f));
    CHECK(boid1.get_velocity().get_x() == doctest::Approx(5.0f));
    CHECK(boid1.get_velocity().get_y() == doctest::Approx(-1.0f));
    CHECK(boid2.get_velocity().get_x() == doctest::Approx(-0.5f));
    CHECK(boid2.get_velocity().get_y() == doctest::Approx(2.5f));
    CHECK(boid3.get_velocity().get_x() == doctest::Approx(3.0f));
    CHECK(boid3.get_velocity().get_y() == doctest::Approx(1.0f));
    CHECK(boid4.get_velocity().get_x() == doctest::Approx(-0.25f));
    CHECK(boid4.get_velocity().get_y() == doctest::Approx(-2.0f));
  }
}
