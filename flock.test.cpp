#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"
#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing three close boids")
{
  const pf::Vector xb1(2., 0.);
  const pf::Vector vb1(0.5, 2.);

  const pf::Vector xb2(-1.5, 1.);
  const pf::Vector vb2(-0.5, -1.);

  const pf::Vector xb3(1., -2.);
  const pf::Vector vb3(1.5, 0.5);

  const pf::Vector xb4(-2., -1.5);
  const pf::Vector vb4(-1., 1.);

  pf::Boid boid1(xb1, vb1);
  pf::Boid boid2(xb2, vb2);
  pf::Boid boid3(xb3, vb3);
  pf::Boid boid4(xb4, vb4);

  const float d       = 25.0f;
  const float ds      = 1.5f;
  const float s       = 1.0f;
  const float a       = 1.0f;
  const float c       = 0.8f;
  const float max_vel = 4.0f;
  const float min_vel = 1.0f;

  pf::Flock flock(d, ds, s, a, c, max_vel, min_vel);

  std::vector<pf::Boid> boids = {boid1, boid2, boid3, boid4};
  flock.add_boids(boids);

  SUBCASE("Testing add_boids method"){
    CHECK(flock.size()==4);
  }

  SUBCASE("Testing add_predator method"){
    
  }
} 