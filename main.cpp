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

  pf::Flock flock(d, ds, s, a, c, 3.0f, 1.5f);
  std::vector<pf::Boid> boids1;
  std::vector<pf::Boid> boids2;
  std::vector<pf::Boid> boids;

  // Create normal boids
  for (int i = 0; i < 30; ++i) {
    pf::Vector pos(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel((rand() % 20 - 10) * 0.1f, (rand() % 20 - 10) * 0.1f);
    boids1.emplace_back(pos, vel); // regular boid
  }

  // Create one predator boid
  pf::Vector predator_pos(400.f, 300.f);
  pf::Vector predator_vel(1.0f, 0.0f);
  pf::Boid predator(predator_pos, predator_vel); // is_predator = true
  boids1.push_back(predator);

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
    flock.flock_update();
    flock.predators_update();

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
    for (auto boid : flock.get_flock()) {
      boid.draw(window);
    }

    for (auto predator : flock.get_predators()) {
      predator.draw(window);
    }

    window.display();
  }

  return 0;
}