#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"
#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing three close boids")
{
  const pf::Vector xb1(1.5, 0.);
  const pf::Vector vb1(0.5, 2.);

  const pf::Vector xb2(0.5, 1.);
  const pf::Vector vb2(-0.5, -1.);

  const pf::Vector xb3(1., -0.5);
  const pf::Vector vb3(1.5, 0.5);

  const pf::Vector xb4(0., -1.5);
  const pf::Vector vb4(-1., 1.);

  pf::Boid boid1(xb1, vb1, 0);
  pf::Boid boid2(xb2, vb2, 0);
  pf::Boid boid3(xb3, vb3, 0);
  pf::Boid boid4(xb4, vb4, 1);

  const float d       = 25.0f;
  const float ds      = 1.5f;
  const float s       = 1.0f;
  const float a       = 1.0f;
  const float c       = 0.8f;
  const float max_vel = 4.0f;
  const float min_vel = 1.0f;

  pf::Flock flock(d, ds, s, a, c, max_vel, min_vel);

  std::vector<pf::Boid> boids1     = {boid1};
  std::vector<pf::Boid> boids2     = {boid2};
  std::vector<pf::Boid> boids3     = {boid3};
  std::vector<pf::Boid> boids4     = {boid4};
  std::vector<pf::Boid> boids      = {boid1, boid2, boid3, boid4};
  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, ds);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, ds);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, ds);
  std::vector<pf::Boid> neighbors4 = boid4.neighboring(boids, ds);

  SUBCASE("Testing add_boids anf size methods")
  {
    flock.add_boids(boids1);
    CHECK(flock.size() == 1);
    flock.add_boids(boids2);
    CHECK(flock.size() == 2);
    flock.add_boids(boids3);
    CHECK(flock.size() == 3);
    flock.add_boids(boids4);
    CHECK(flock.size() == 4);
  }

  SUBCASE("Testing get_flock method")
  {
    flock.add_boids(boids);
    auto flock_2 = flock.get_flock();

    CHECK(flock.size() == 4);
    CHECK(flock_2.size() == 3);

    flock_2.pop_back();

    CHECK(flock_2.size() == 2);
    CHECK(flock.size() == 4);
  }

  SUBCASE("Testing get_predators method")
  {
    flock.add_boids(boids);
    auto flock_2 = flock.get_predators();

    CHECK(flock.size() == 4);
    CHECK(flock_2.size() == 1);

    flock_2.pop_back();

    CHECK(flock_2.size() == 0);
    CHECK(flock.size() == 4);
  }

  SUBCASE("Testing flock_separation method")
  {
    flock.add_boids(boids);

    CHECK(flock.flock_separation(boid1, boids).get_x()
          == doctest::Approx(1.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid1, boids).get_y()
          == doctest::Approx(-0.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid3, boids).get_x()
          == doctest::Approx(0.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid3, boids).get_y()
          == doctest::Approx(0.5).epsilon(0.1));
  }

  SUBCASE("Testing flock_alignment method")
  {
    flock.add_boids(boids);

    CHECK(flock.flock_alignment(boid1, neighbors1).get_x()
          == doctest::Approx(0.).epsilon(0.01));
    CHECK(flock.flock_alignment(boid1, neighbors1).get_y()
          == doctest::Approx(-2.25).epsilon(0.01));
    CHECK(flock.flock_alignment(boid2, neighbors2).get_x()
          == doctest::Approx(1.).epsilon(0.1));
    CHECK(flock.flock_alignment(boid2, neighbors2).get_y()
          == doctest::Approx(3.).epsilon(0.1));
    CHECK(flock.flock_alignment(boid4, neighbors4).get_x()
          == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_alignment(boid4, neighbors4).get_y()
          == doctest::Approx(0.).epsilon(0.1));
  }

  SUBCASE("Testing flock_cohesion method")
  {
    flock.add_boids(boids);

    CHECK(flock.flock_cohesion(boid1, neighbors1).get_x()
          == doctest::Approx(-0.6).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid1, neighbors1).get_y()
          == doctest::Approx(0.2).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid2, neighbors2).get_x()
          == doctest::Approx(0.8).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid2, neighbors2).get_y()
          == doctest::Approx(-0.8).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid4, neighbors4).get_x()
          == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid4, neighbors4).get_y()
          == doctest::Approx(0.).epsilon(0.1));
  }

  SUBCASE("Testing avoid_predators method")
  {
    const pf::Vector xb5(15, 20);
    const pf::Vector vb5(-1., 1.);
    pf::Boid boid5(xb5, vb5, 0);
    std::vector<pf::Boid> boids5 = {boid5};

    flock.add_boids(boids);
    flock.add_boids(boids5);

    CHECK(flock.avoid_predators(boid1).get_x()
          == doctest::Approx(10.607).epsilon(0.001));
    CHECK(flock.avoid_predators(boid1).get_y()
          == doctest::Approx(10.607).epsilon(0.001));
    CHECK(flock.avoid_predators(boid2).get_x()
          == doctest::Approx(2.942).epsilon(0.001));
    CHECK(flock.avoid_predators(boid2).get_y()
          == doctest::Approx(14.709).epsilon(0.001));
    CHECK(flock.avoid_predators(boid3).get_x()
          == doctest::Approx(17.678).epsilon(0.001));
    CHECK(flock.avoid_predators(boid3).get_y()
          == doctest::Approx(17.678).epsilon(0.001));
    CHECK(flock.avoid_predators(boid5).get_x()
          == doctest::Approx(0.).epsilon(0.001));
    CHECK(flock.avoid_predators(boid5).get_y()
          == doctest::Approx(0.).epsilon(0.001));
  }

  SUBCASE("Testing chase_prey method")
  {
    const pf::Vector xb5(0.5, 0.5);
    const pf::Vector vb5(0.5, -1.);

    const pf::Vector xb6(5., 2.);
    const pf::Vector vb6(2, -1.5);

    pf::Boid boid5(xb5, vb5, 1);
    pf::Boid boid6(xb6, vb6, 1);
    std::vector<pf::Boid> boids_2 = {boid1, boid2, boid3, boid4, boid5, boid6};

    flock.add_boids(boids_2);

    std::vector<pf::Boid> neighbors5   = boid5.neighboring(boids_2, ds);
    std::vector<pf::Boid> neighbors6   = boid6.neighboring(boids_2, ds);
    std::vector<pf::Boid> neighbors4_2 = boid4.neighboring(boids_2, ds);

    CHECK(flock.chase_prey(boid4, neighbors4_2).get_x()
          == doctest::Approx(7.071).epsilon(0.001));
    CHECK(flock.chase_prey(boid4, neighbors4_2).get_y()
          == doctest::Approx(7.071).epsilon(0.001));
    CHECK(flock.chase_prey(boid5, neighbors5).get_x()
          == doctest::Approx(0.).epsilon(0.001));
    CHECK(flock.chase_prey(boid5, neighbors5).get_y()
          == doctest::Approx(10.).epsilon(0.001));
    CHECK(flock.chase_prey(boid6, neighbors6).get_x()
          == doctest::Approx(0.).epsilon(0.001));
    CHECK(flock.chase_prey(boid6, neighbors6).get_y()
          == doctest::Approx(0.).epsilon(0.001));
  }
} 