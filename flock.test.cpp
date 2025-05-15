#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"
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

  const pf::Vector xb4(2.0, 2.0);
  const pf::Vector vb4(2.0, 2.0);

  const pf::Vector xb5(2.0, 1.0);
  const pf::Vector vb5(2.0, 2.0);

  pf::Boid boid1(xb1, vb1, 0);
  pf::Boid boid2(xb2, vb2, 0);
  pf::Boid boid3(xb3, vb3, 0);
  pf::Boid boid4(xb4, vb4, 0);
  pf::Boid boid5(xb5, vb5, 0);

  pf::Flock flock1;
  pf::Flock flock2;

  flock1.add_boids({boid1, boid2, boid3});
  flock2.add_boids({boid4, boid5});

  SUBCASE("Testing add_boids")
  {
    CHECK(flock1.size() == 3);
    CHECK(flock2.size() == 2);
  }
}