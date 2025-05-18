#include "flock.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

bool valid_int(const std::string& input, int& value)
{
  std::stringstream ss(input);
  char leftover;

  if (!(ss >> value) || (ss >> leftover)) {
    return false;
  }
  return true;
}

int main()
{
  std::cout << "Input number of boids (from 1 to 50): \n";
  std::string boids;
  int nb;
  std::getline(std::cin, boids);
  if (!valid_int(boids, nb) || nb < 1 || nb > 50) {
    std::cerr
        << "Error: number of boids must be an integer between 1 and 50 \n";
    return 1;
  }

  std::cout << "Input number of predators (from 0 to 10): \n";
  std::string predators;
  int np;
  std::getline(std::cin, predators);
  if (!valid_int(predators, np) || np < 0 || np > 5) {
    std::cerr
        << "Error: number of predators must be an integer between 0 and 5 \n";
    return 1;
  }

  std::cout
      << "Input range of view of the boids (any number between 50 and 100): \n";
  float d;
  std::cin >> d;
  if (d < 50.0f || d > 100.0f) {
    std::cerr << "Error: input value outside of permitted range \n";
    return 1;
  }

  std::cout << "Input protected range of the boids (any number between 10 and "
               "15): \n";
  float ds;
  std::cin >> ds;
  if (ds < 10.0f || ds > 15.0f) {
    std::cerr << "Error: input value outside of permitted range \n";
    return 1;
  }

  std::cout << "Input protected range repulsion of the boids (any number "
               "between 0.1 and 0.8): \n";
  float s;
  std::cin >> s;
  if (s < 0.1f || s > 0.8f) {
    std::cerr << "Error: input value outside of permitted range \n";
    return 1;
  }

  std::cout
      << "Input alignment parameter of the boids (any number between 0.05 "
         "and 0.1): \n";
  float a;
  std::cin >> a;
  if (a < 0.05f || a > 0.1f) {
    std::cerr << "Error: input value outside of permitted range \n";
    return 1;
  }

  std::cout << "Input cohesion parameter of the boids (any number between "
               "0.002 and 0.004): \n";
  float c;
  std::cin >> c;
  if (c < 0.002f || c > 0.004f) {
    std::cerr << "Error: input value outside of permitted range \n";
    return 1;
  }

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
    boids1.emplace_back(pos_b, vel_b, false);
  }

  // Create n predators
  for (int i = 0; i < np; ++i) {
    pf::Vector pos_p(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel_p(((rand() % 20 - 10) * 0.1f) + 100.0f,
                     ((rand() % 20 - 10) * 0.1f) + 100.0f);
    boids1.emplace_back(pos_p, vel_p, true);
  }

  // Add all boids to the flock
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