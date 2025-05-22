#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"
#include "boid.hpp"
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Flock Class")
{
  const pf::Vector xb1(15, 0.);
  const pf::Vector vb1(0.5, 2.);

  const pf::Vector xb2(-5., 10.);
  const pf::Vector vb2(-0.5, -1.);

  const pf::Vector xb3(12., -11);
  const pf::Vector vb3(1.5, 0.5);

  const pf::Vector xb4(-6., 7);
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
  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, d);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, d);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, d);
  std::vector<pf::Boid> neighbors4 = boid4.neighboring(boids, d);

  SUBCASE("Testing add_boids and size methods")
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
    const pf::Vector xb1_2(1.5, 0.);
    const pf::Vector vb1_2(0.5, 2.);

    const pf::Vector xb2_2(0.5, 1.);
    const pf::Vector vb2_2(-0.5, -1.);

    const pf::Vector xb3_2(1., -0.5);
    const pf::Vector vb3_2(1.5, 0.5);

    const pf::Vector xb4_2(0., -1.5);
    const pf::Vector vb4_2(-1., 1.);

    pf::Boid boid1_2(xb1_2, vb1_2, 0);
    pf::Boid boid2_2(xb2_2, vb2_2, 0);
    pf::Boid boid3_2(xb3_2, vb3_2, 0);
    pf::Boid boid4_2(xb4_2, vb4_2, 1);

    std::vector<pf::Boid> boids_2 = {boid1_2, boid2_2, boid3_2, boid4_2};

    flock.add_boids(boids_2);

    CHECK(flock.flock_separation(boid1_2, boids_2).get_x()
          == doctest::Approx(1.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid1_2, boids_2).get_y()
          == doctest::Approx(-0.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid3_2, boids_2).get_x()
          == doctest::Approx(0.5).epsilon(0.1));
    CHECK(flock.flock_separation(boid3_2, boids_2).get_y()
          == doctest::Approx(0.5).epsilon(0.1));
  }

  SUBCASE("Testing flock_alignment method")
  {
    flock.add_boids(boids);

    CHECK(flock.flock_alignment(boid1, neighbors1).get_x()
          == doctest::Approx(-0.5).epsilon(0.1));
    CHECK(flock.flock_alignment(boid1, neighbors1).get_y()
          == doctest::Approx(-1.833).epsilon(0.001));
    CHECK(flock.flock_alignment(boid2, neighbors2).get_x()
          == doctest::Approx(0.25).epsilon(0.01));
    CHECK(flock.flock_alignment(boid2, neighbors2).get_y()
          == doctest::Approx(2.5).epsilon(0.1));
    CHECK(flock.flock_alignment(boid4, neighbors4).get_x()
          == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_alignment(boid4, neighbors4).get_y()
          == doctest::Approx(0.).epsilon(0.1));
  }

  SUBCASE("Testing flock_cohesion method")
  {
    flock.add_boids(boids);

    CHECK(flock.flock_cohesion(boid1, neighbors1).get_x()
          == doctest::Approx(-11.7).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid1, neighbors1).get_y()
          == doctest::Approx(1.6).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid2, neighbors2).get_x()
          == doctest::Approx(7.6).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid2, neighbors2).get_y()
          == doctest::Approx(-5.2).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid4, neighbors4).get_x()
          == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_cohesion(boid4, neighbors4).get_y()
          == doctest::Approx(0.).epsilon(0.1));
  }

  SUBCASE("Testing avoid_predators method")
  {
    const pf::Vector xb5(-5.5, 7);
    const pf::Vector vb5(2., 0.5);
    pf::Boid boid5(xb5, vb5, 0);
    std::vector<pf::Boid> boids5 = {boid5};

    flock.add_boids(boids);
    flock.add_boids(boids5);

    CHECK(flock.avoid_predators(boid1).get_x()
          == doctest::Approx(14.23).epsilon(0.01));
    CHECK(flock.avoid_predators(boid1).get_y()
          == doctest::Approx(-4.74).epsilon(0.01));
    CHECK(flock.avoid_predators(boid2).get_x()
          == doctest::Approx(4.74).epsilon(0.01));
    CHECK(flock.avoid_predators(boid2).get_y()
          == doctest::Approx(14.23).epsilon(0.01));
    CHECK(flock.avoid_predators(boid3).get_x()
          == doctest::Approx(0.).epsilon(0.01));
    CHECK(flock.avoid_predators(boid3).get_y()
          == doctest::Approx(0.).epsilon(0.01));
    CHECK(flock.avoid_predators(boid5).get_x()
          == doctest::Approx(25.).epsilon(0.01));
    CHECK(flock.avoid_predators(boid5).get_y()
          == doctest::Approx(0.).epsilon(0.01));
  }

  SUBCASE("Testing chase_prey method")
  {
    const pf::Vector xb5(10., 4.);
    const pf::Vector vb5(2, 0.5);

    const pf::Vector xb6(60., 40.);
    const pf::Vector vb6(1.5, -2.);

    pf::Boid boid5(xb5, vb5, 1);
    pf::Boid boid6(xb6, vb6, 1);
    std::vector<pf::Boid> boids_2 = {boid1, boid2, boid3, boid4, boid5, boid6};

    flock.add_boids(boids_2);

    std::vector<pf::Boid> neighbors5   = boid5.neighboring(boids_2, d);
    std::vector<pf::Boid> neighbors6   = boid6.neighboring(boids_2, d);
    std::vector<pf::Boid> neighbors4_2 = boid4.neighboring(boids_2, d);

    CHECK(flock.chase_prey(boid4, neighbors4_2).get_x()
          == doctest::Approx(3.16).epsilon(0.01));
    CHECK(flock.chase_prey(boid4, neighbors4_2).get_y()
          == doctest::Approx(9.49).epsilon(0.01));
    CHECK(flock.chase_prey(boid5, neighbors5).get_x()
          == doctest::Approx(7.81).epsilon(0.01));
    CHECK(flock.chase_prey(boid5, neighbors5).get_y()
          == doctest::Approx(-6.25).epsilon(0.01));
    CHECK(flock.chase_prey(boid6, neighbors6).get_x()
          == doctest::Approx(0.).epsilon(0.01));
    CHECK(flock.chase_prey(boid6, neighbors6).get_y()
          == doctest::Approx(0.).epsilon(0.01));
  }

  SUBCASE("Testing Statistics ")
  {
    std::vector<pf::Boid> boids_0 = {boid1};

    flock.add_boids(boids_0);

    CHECK(flock.flock_state().average_dist == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_state().dev_dist == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_state().average_vel == doctest::Approx(0.).epsilon(0.1));
    CHECK(flock.flock_state().dev_vel == doctest::Approx(0.).epsilon(0.1));

    std::vector<pf::Boid> boids_2 = {boid2, boid3, boid4};
    flock.add_boids(boids_2);

    CHECK(flock.flock_state().average_dist
          == doctest::Approx(18.6).epsilon(0.1));
    CHECK(flock.flock_state().dev_dist == doctest::Approx(8.5).epsilon(0.1));
    CHECK(flock.flock_state().average_vel == doctest::Approx(1.5).epsilon(0.1));
    CHECK(flock.flock_state().dev_vel == doctest::Approx(0.3).epsilon(0.1));
  }
}

TEST_CASE("Testing predators_update")
{
  const pf::Vector xb1(17., 9.);
  const pf::Vector vb1(2, 1.5);

  const pf::Vector xb2(16., 9.);
  const pf::Vector vb2(1.5, -2.);

  const pf::Vector xb3(20., 8.);
  const pf::Vector vb3(3., 1.);

  const pf::Vector xb4(18., 5.);
  const pf::Vector vb4(2.5, -3.);

  const pf::Vector xb5(720., 510.);
  const pf::Vector vb5(2, 1.5);

  const pf::Vector xb6(150., 200.);
  const pf::Vector vb6(-3, 1.5);

  pf::Boid boid1(xb1, vb1, 1);
  pf::Boid boid2(xb2, vb2, 1);
  pf::Boid boid3(xb3, vb3, 0);
  pf::Boid boid4(xb4, vb4, 0);
  pf::Boid boid5(xb5, vb5, 1);
  pf::Boid boid6(xb6, vb6, 1);

  const float d       = 25.0f;
  const float ds      = 1.5f;
  const float s       = 1.0f;
  const float a       = 1.0f;
  const float c       = 0.8f;
  const float max_vel = 4.0f;
  const float min_vel = 1.0f;
  const float delta_t = 1.0f;

  pf::Flock flock(d, ds, s, a, c, max_vel, min_vel);

  std::vector<pf::Boid> boids = {boid1, boid2, boid3, boid4, boid5, boid6};

  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, d);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, d);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, d);
  std::vector<pf::Boid> neighbors4 = boid4.neighboring(boids, d);
  std::vector<pf::Boid> neighbors5 = boid5.neighboring(boids, d);
  std::vector<pf::Boid> neighbors6 = boid6.neighboring(boids, d);

  flock.add_boids(boids);
  flock.predators_update(delta_t);

  auto predators = flock.get_predators();

  CHECK(predators[0].get_velocity().get_x()
        == doctest::Approx(16.47).epsilon(0.01));
  CHECK(predators[0].get_velocity().get_y()
        == doctest::Approx(11.97).epsilon(0.01));
  CHECK(predators[0].get_position().get_x()
        == doctest::Approx(33.47).epsilon(0.01));
  CHECK(predators[0].get_position().get_y()
        == doctest::Approx(20.97).epsilon(0.01));

  CHECK(predators[2].get_velocity().get_x()
        == doctest::Approx(-10.5).epsilon(0.01));
  CHECK(predators[2].get_velocity().get_y()
        == doctest::Approx(-11.0).epsilon(0.01));
  CHECK(predators[2].get_position().get_x()
        == doctest::Approx(709.5).epsilon(0.01));
  CHECK(predators[2].get_position().get_y()
        == doctest::Approx(499.).epsilon(0.01));

  CHECK(predators[3].get_velocity().get_x()
        == doctest::Approx(-3.).epsilon(0.01));
  CHECK(predators[3].get_velocity().get_y()
        == doctest::Approx(1.5).epsilon(0.01));
  CHECK(predators[3].get_position().get_x()
        == doctest::Approx(147).epsilon(0.01));
  CHECK(predators[3].get_position().get_y()
        == doctest::Approx(201.5).epsilon(0.01));
}

TEST_CASE("Testing flock_update")
{
  const pf::Vector xb1(10.5, 12.);
  const pf::Vector vb1(1, -1.5);

  const pf::Vector xb2(11., 11.5);
  const pf::Vector vb2(-0.5, -1.);

  const pf::Vector xb3(0.5, 2.);
  const pf::Vector vb3(2., 0.);

  const pf::Vector xb4(200., 180.);
  const pf::Vector vb4(-2, 1.);

  const pf::Vector xb5(710., 505.);
  const pf::Vector vb5(1.5, 0.5);

  const pf::Vector xb6(10., 12.);
  const pf::Vector vb6(0.5, -0.5);

  const pf::Vector xb7(16., 14.);
  const pf::Vector vb7(0., -2.);

  pf::Boid boid1(xb1, vb1, 0);
  pf::Boid boid2(xb2, vb2, 0);
  pf::Boid boid3(xb3, vb3, 0);
  pf::Boid boid4(xb4, vb4, 0);
  pf::Boid boid5(xb5, vb5, 0);
  pf::Boid boid6(xb6, vb6, 1);
  pf::Boid boid7(xb7, vb7, 1);

  const float d       = 25.0f;
  const float ds      = 1.5f;
  const float s       = 1.0f;
  const float a       = 1.0f;
  const float c       = 0.8f;
  const float max_vel = 4.0f;
  const float min_vel = 1.0f;
  const float delta_t = 1.0f;

  pf::Flock flock(d, ds, s, a, c, max_vel, min_vel);

  std::vector<pf::Boid> boids = {boid1, boid2, boid3, boid4,
                                 boid5, boid6, boid7};

  std::vector<pf::Boid> neighbors1 = boid1.neighboring(boids, d);
  std::vector<pf::Boid> neighbors2 = boid2.neighboring(boids, d);
  std::vector<pf::Boid> neighbors3 = boid3.neighboring(boids, d);
  std::vector<pf::Boid> neighbors4 = boid4.neighboring(boids, d);
  std::vector<pf::Boid> neighbors5 = boid5.neighboring(boids, d);
  std::vector<pf::Boid> neighbors6 = boid6.neighboring(boids, d);
  std::vector<pf::Boid> neighbors7 = boid7.neighboring(boids, d);

  flock.add_boids(boids);
  flock.flock_update(delta_t);

  auto boids_0 = flock.get_flock();

  CHECK(boids_0[0].get_velocity().get_x()
        == doctest::Approx(15.08).epsilon(0.01));
  CHECK(boids_0[0].get_velocity().get_y()
        == doctest::Approx(9.44).epsilon(0.01));
  CHECK(boids_0[0].get_position().get_x()
        == doctest::Approx(25.58).epsilon(0.01));
  CHECK(boids_0[0].get_position().get_y()
        == doctest::Approx(21.44).epsilon(0.01));

  CHECK(boids_0[3].get_velocity().get_x()
        == doctest::Approx(-2.0).epsilon(0.01));
  CHECK(boids_0[3].get_velocity().get_y()
        == doctest::Approx(1.0).epsilon(0.01));
  CHECK(boids_0[3].get_position().get_x()
        == doctest::Approx(198.0).epsilon(0.01));
  CHECK(boids_0[3].get_position().get_y()
        == doctest::Approx(181.0).epsilon(0.01));

  CHECK(boids_0[4].get_velocity().get_x()
        == doctest::Approx(-11).epsilon(0.01));
  CHECK(boids_0[4].get_velocity().get_y()
        == doctest::Approx(-12).epsilon(0.01));
  CHECK(boids_0[4].get_position().get_x()
        == doctest::Approx(699.).epsilon(0.01));
  CHECK(boids_0[4].get_position().get_y()
        == doctest::Approx(493).epsilon(0.01));
}