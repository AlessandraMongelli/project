#include "flock.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

int main()
{
  std::cout << "Input number of boids (from 0 to 100): \n";
  int nb;
  std::cin >> nb;
  std::cout << "Input number of predators (from 0 to 10): \n";
  int np;
  std::cin >> np;
  std::cout << "Input range of view of the boids: \n";
  float d;
  std::cin >> d;
  std::cout << "Input protected range of the boids: \n";
  float ds;
  std::cin >> ds;
  std::cout << "Input protected range repulsion of the boids: \n";
  float s;
  std::cin >> s;
  std::cout << "Input alignment parameter of the boids: \n";
  float a;
  std::cin >> a;
  std::cout << "Input cohesion parameter of the boids: \n";
  float c;
  std::cin >> c;

  sf::Clock delay_clock;

  while (delay_clock.getElapsedTime().asSeconds() < 3.0f) {
    std::cout << "The simulation will start in "
              << (3 - (int)delay_clock.getElapsedTime().asSeconds())
              << " seconds...\r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << '\n';
  std::cout << '\n';

  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
  window.setFramerateLimit(60);

  pf::Flock flock(d, ds, s, a, c, 200.0f, 100.0f);
  std::vector<pf::Boid> boids1;

  // Create n normal boids
  for (int i = 0; i < nb; ++i) {
    pf::Vector pos_b(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel_b(((rand() % 20 - 10) * 0.1f) + 100.0f,
                     ((rand() % 20 - 10) * 0.1f) + 100.0f);
    boids1.emplace_back(pos_b, vel_b, false); // regular boids
  }

  // Create n predators
  for (int i = 0; i < np; ++i) {
    pf::Vector pos_p(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel_p(((rand() % 20 - 10) * 0.1f) + 100.0f,
                     ((rand() % 20 - 10) * 0.1f) + 100.0f);
    boids1.emplace_back(pos_p, vel_p, true); // predators
  }

  // Add all boids to their relative flock
  flock.add_boids(boids1);

  sf::Clock clock;
  sf::Clock statistics_clock;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    float delta_t = clock.restart().asSeconds();

    // Update flock and predator
    flock.flock_update(delta_t);
    flock.predators_update(delta_t);

    sf::Time time_passed = statistics_clock.getElapsedTime();

    const pf::Statistics flock_state = flock.flock_state();

    if (flock.size() >= 2 && time_passed.asSeconds() >= 2.0f) {
      std::cout << "Average velocity: " << flock_state.average_vel << " +/- "
                << flock_state.dev_vel << ";       "
                << "Medium distance among boids: " << flock_state.average_dist
                << " +/- " << flock_state.dev_dist << ";\n";

      statistics_clock.restart();
    }

    window.clear(sf::Color::Black);

    // Draw boids and a predator
    for (const auto& boid : flock.get_flock()) {
      boid.draw(window);
    }

    for (const auto& boid : flock.get_predators()) {
      boid.draw(window);
    }

    window.display();
  }

  return 0;
}