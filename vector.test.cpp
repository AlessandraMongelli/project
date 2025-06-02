#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "vector.hpp"
#include "doctest.h"

TEST_CASE("Testing get_...()")
{
  pf::Vector v{3.12f, 6.99f};

  CHECK(v.get_x() == doctest::Approx(3.12).epsilon(0.01));
  CHECK(v.get_y() == doctest::Approx(6.99).epsilon(0.01));
}

TEST_CASE("Testing set_...()")
{
  pf::Vector v{3.12f, 6.99f};

  v.set_x(6.11f);
  v.set_y(0.81f);
  CHECK(v.get_x() == doctest::Approx(6.11).epsilon(0.1));
  CHECK(v.get_y() == doctest::Approx(0.81).epsilon(0.1));
}

TEST_CASE("Testing operator+")
{
  SUBCASE("Positive components")
  {
    const pf::Vector v{1.0f, 1.0f};
    const pf::Vector w{1.5f, 2.0f};
    const pf::Vector sum{w + v};

    CHECK(sum.get_x() == doctest::Approx(2.5).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(3.0).epsilon(0.1));
  };

  SUBCASE("Negative components")
  {
    const pf::Vector v{-1.2f, -3.0f};
    const pf::Vector w{-1.5f, -2.5f};
    const pf::Vector sum{w + v};

    CHECK(sum.get_x() == doctest::Approx(-2.7).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(-5.5).epsilon(0.1));
  };

  SUBCASE("Null components")
  {
    const pf::Vector v{1.0f, 0.0f};
    const pf::Vector w{0.0f, 2.0f};
    const pf::Vector sum{w + v};

    CHECK(sum.get_x() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(sum.get_y() == doctest::Approx(2.0).epsilon(0.1));
  };
}

TEST_CASE("Testing operator +=")
{
  SUBCASE("Positive Components")
  {
    pf::Vector v{1.f, 2.f};
    const pf::Vector w{3.f, 2.f};
    v += w;
    CHECK(v.get_x() == doctest::Approx(4.0).epsilon(0.1));
    CHECK(v.get_y() == doctest::Approx(4.0).epsilon(0.1));
  };

  SUBCASE("Negative Components")
  {
    pf::Vector v{-1.f, -3.f};
    const pf::Vector w{-2.f, -2.f};
    v += w;
    CHECK(v.get_x() == doctest::Approx(-3.0).epsilon(0.1));
    CHECK(v.get_y() == doctest::Approx(-5.0).epsilon(0.1));
  };

  SUBCASE("Null Components")
  {
    pf::Vector v{0.f, 2.f};
    const pf::Vector w{3.f, 0.f};
    v += w;
    CHECK(v.get_x() == doctest::Approx(3.0).epsilon(0.1));
    CHECK(v.get_y() == doctest::Approx(2.0).epsilon(0.1));
  };
}

TEST_CASE("Testing operator-")
{
  SUBCASE("Positive Components")
  {
    const pf::Vector v{1.0f, 5.0f};
    const pf::Vector w{2.0f, 1.5f};
    const pf::Vector diff{w - v};

    CHECK(diff.get_x() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(diff.get_y() == doctest::Approx(-3.5).epsilon(0.1));
  };

  SUBCASE("Negative Components")
  {
    const pf::Vector v{-2.5f, -1.5f};
    const pf::Vector w{-3.2f, -1.3f};
    const pf::Vector diff{w - v};

    CHECK(diff.get_x() == doctest::Approx(-0.7).epsilon(0.1));
    CHECK(diff.get_y() == doctest::Approx(0.2).epsilon(0.1));
  };

  SUBCASE("Null Components")
  {
    const pf::Vector v{0.0f, 1.0f};
    const pf::Vector w{2.0f, 0.0f};
    const pf::Vector diff{w - v};

    CHECK(diff.get_x() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(diff.get_y() == doctest::Approx(-1.0).epsilon(0.1));
  };
}

TEST_CASE("Testing operator*")
{
  SUBCASE("Positive scalar")
  {
    const pf::Vector v{1.0f, 0.0f};
    float x = 2.5;
    const pf::Vector prod{v * x};

    CHECK(prod.get_x() == doctest::Approx(2.5).epsilon(0.1));
    CHECK(prod.get_y() == doctest::Approx(0.0).epsilon(0.1));
  };

  SUBCASE("Negative scalar")
  {
    const pf::Vector v{-3.0f, 2.0f};
    float x = -0.5;
    const pf::Vector prod{v * x};

    CHECK(prod.get_x() == doctest::Approx(1.5).epsilon(0.1));
    CHECK(prod.get_y() == doctest::Approx(-1.0).epsilon(0.1));
  };

  SUBCASE("Null scalar")
  {
    const pf::Vector v{-1.0f, 2.0f};
    float x = -0.0;
    const pf::Vector prod{v * x};

    CHECK(prod.get_x() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(prod.get_y() == doctest::Approx(0.0).epsilon(0.1));
  };
}

TEST_CASE("Testing distance")
{
  SUBCASE("Positive components")
  {
    const pf::Vector v{1.0f, 1.0f};
    const pf::Vector w{2.0f, 1.0f};

    CHECK(v.distance(w) == doctest::Approx(1.0).epsilon(0.1));
  };

  SUBCASE("Negative components")
  {
    const pf::Vector v{-2.0f, -1.0f};
    const pf::Vector w{-1.0f, -1.0f};

    CHECK(v.distance(w) == doctest::Approx(1.0).epsilon(0.1));
  };

  SUBCASE("Null components")
  {
    const pf::Vector v{1.0f, 0.0f};
    const pf::Vector w{0.0f, 0.0f};

    CHECK(v.distance(w) == doctest::Approx(1.0).epsilon(0.1));
  };
}

TEST_CASE("Testing norm")
{
  SUBCASE("Positive components")
  {
    const pf::Vector v{1.0f, 2.0f};

    CHECK(v.norm() == doctest::Approx(2.2).epsilon(0.1));
  };

  SUBCASE("Negative components")
  {
    const pf::Vector v{-2.0f, -1.0f};

    CHECK(v.norm() == doctest::Approx(2.2).epsilon(0.1));
  };

  SUBCASE("Null components")
  {
    const pf::Vector v{0.0f, 0.0f};

    CHECK(v.norm() == doctest::Approx(0.0).epsilon(0.1));
  };
}

TEST_CASE("Testing product")
{
  SUBCASE("Positive Components")
  {
    const pf::Vector v{1.0f, 2.0f};
    const pf::Vector w{2.0f, 1.3f};

    CHECK(v.product(w) == doctest::Approx(4.6).epsilon(0.1));
  };

  SUBCASE("Negative Components")
  {
    const pf::Vector v{-1.5f, -2.0f};
    const pf::Vector w{-2.0f, -1.0f};

    CHECK(v.product(w) == doctest::Approx(5.0).epsilon(0.1));
  };

  SUBCASE("Null Components")
  {
    const pf::Vector v{1.0f, 0.0f};
    const pf::Vector w{0.0f, 0.5f};

    CHECK(v.product(w) == doctest::Approx(0.0).epsilon(0.1));
  };
}

TEST_CASE("Testing operator ==")
{
  const pf::Vector v{-2.0f, 1.3f};
  const pf::Vector w{-2.0f, 1.3f};

  CHECK(v == w);
}
