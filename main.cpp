#include "boid.hpp"
#include "flock.hpp"
#include "vector.hpp"
#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

/* void print_boid_statistics(const pf::Flock& flock)
{
  std::vector<pf::Boid> boids = flock.get_flock();
  if (boids.size() < 2)
    return;

  float sum_dist = 0.0f, sum_dist2 = 0.0f;
  for (size_t i = 0; i < boids.size(); ++i) {
    for (size_t j = i + 1; j < boids.size(); ++j) {
      float dist = boids[i].get_position().distance(boids[j].get_position());
      sum_dist += dist;
      sum_dist2 += std::pow(dist, 2);
    }
  }

  float n = static_cast<float>(boids.size() * (boids.size() - 1)) / 2.0f;
  float medium_dist  = sum_dist / n;
  float medium_dist2 = sum_dist2 / n;
  float dev_dist     = std::sqrt(medium_dist2 - std::pow(medium_dist, 2));

  float sum_vel = 0.0f, sum_vel2 = 0.0f;
  for (const auto& boid : boids) {
    float speed = boid.get_velocity().norm();
    sum_vel += speed;
    sum_vel2 += std::pow(speed, 2);
  }

  float medium_vel  = sum_vel / boids.size();
  float medium_vel2 = sum_vel2 / boids.size();
  float dev_vel     = std::sqrt(medium_vel2 - std::pow(medium_vel, 2));

  std::cout << "La distanza media tra i boids ± deviazione standard: "
            << medium_dist << " ± " << dev_dist << '\n'
            << "La velocità media dei boids ± deviazione standard: "
            << medium_vel << " ± " << dev_vel << '\n';
} */

int main()
{
  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
  window.setFramerateLimit(60);

  pf::Flock flock1;
  pf::Flock flock2;

  std::vector<pf::Boid> boids1;
  std::vector<pf::Boid> boids2;
  std::vector<pf::Boid> boids;

  // Create normal boids
  for (int i = 0; i < 50; ++i) {
    pf::Vector pos(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel((rand() % 20 - 10) * 0.1f, (rand() % 20 - 10) * 0.1f);
    boids1.emplace_back(pos, vel, false); // regular boid
  }

  /* for (int i = 0; i < 15; ++i) {
    pf::Vector pos(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel((rand() % 20 - 10) * 0.1f, (rand() % 20 - 10) * 0.1f);
    boids2.emplace_back(pos, vel, false); // regular boid
  } */

  // Create one predator boid
  pf::Vector predator_pos(400.f, 300.f);
  pf::Vector predator_vel(1.0f, 0.0f);
  pf::Boid predator(predator_pos, predator_vel, true); // is_predator = true
  boids1.push_back(predator);

  // Add all boids to their relative flock
  flock1.add_boids(boids1);
  // flock2.add_boids(boids2);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    window.clear(sf::Color::Black);

    // print_boid_statistics(flock1);

    // Update flock (predator is handled separately inside flock_update)
    flock1.flock_update(60.0f, 8.0f, 0.08f, 0.05f, 0.005f, 3.0f, 1.5f, 30.0f);

    // flock2.flock_update(40.0f, 9.0f, 0.08f, 0.05f, 0.009f, 3.0f, 1.5f);

    flock1.predators_update(40.0f, 3.0f, 1.5f, 0.5f);

    // Draw boids
    for (const auto& boid :
         const_cast<std::vector<pf::Boid>&>(flock1.get_flock())) {
      const_cast<pf::Boid&>(boid).draw(window);
    }

    /* for (const auto& boid :
         const_cast<std::vector<pf::Boid>&>(flock2.get_flock())) {
      const_cast<pf::Boid&>(boid).draw(window);
    } */

    // Draw predators
    for (auto& predator :
         const_cast<std::vector<pf::Boid>&>(flock1.get_predators())) {
      predator.draw(window);
    }

    window.display();
  }

  return 0;
}