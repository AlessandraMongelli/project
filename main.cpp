#include "boid.hpp"
#include "vector.hpp"
#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

void update_boids(std::vector<pf::Boid>& boids, float separation_dist,
                  float alignment_factor, float cohesion_factor)
{
    float sum_dist  = 0.0f;
    float sum_dist2 = 0.0f;
    for (int i = 0; i < boids.size(); ++i) {
      for (int j = i + 1; j < boids.size(); ++j) {
        float dist = boids[i].get_position().distance(boids[j].get_position());
        sum_dist += dist;
        sum_dist2 += std::pow(dist, 2);
      }
    }
    const float medium_dist =
        sum_dist / ((boids.size() * (boids.size() - 1)) / 2);
    const float medium_dist2 =
        sum_dist2 / ((boids.size() * (boids.size() - 1)) / 2);
    const float dev_dist = std::sqrt(medium_dist2 - std::pow(medium_dist, 2));

    float sum_vel  = 0.0f;
    float sum_vel2 = 0.0f;
    for (int i = 0; i < boids.size(); ++i) {
      sum_vel += boids[i].get_velocity().norm();
      sum_vel2 += std::pow(boids[i].get_velocity().norm(), 2);
    }
    const float medium_vel =
        sum_vel / ((boids.size() * (boids.size() - 1)) / 2);
    const float medium_vel2 =
        sum_vel2 / ((boids.size() * (boids.size() - 1)) / 2);
    const float dev_vel = std::sqrt(medium_vel2 - std::pow(medium_vel, 2));

    std::cout << "La distanza media tra i boids con la sua rispettiva '\n' "
                 "deviazione standard è "
              << medium_dist << " ± " << dev_dist << '\n'
              << "La velocità media dei boids con la sua rispettiva '\n' "
                 "deviazione standard è "
              << medium_vel << " ± " << dev_vel << '\n';
    
    for (auto& boid : boids) {
    std::vector<pf::Boid> neighbors = boid.neighboring(boids, separation_dist);

    // Calculate separation, alignment, and cohesion behaviors
    pf::Vector separation =
        boid.separation(neighbors, 8.0f, 0.08f); // Example parameters
    pf::Vector alignment = boid.alignment(neighbors, alignment_factor);
    pf::Vector cohesion  = boid.cohesion(neighbors, cohesion_factor);

    // Update the velocity by combining the behaviors
    pf::Vector delta_v = separation + alignment + cohesion;

    // Update and clamp velocity
    boid.update_velocity(delta_v);
    boid.speed_limit(3.0f, 1.5f); 
    
    // Update boid's position based on the new velocity
    boid.update_position(boid.get_velocity());

    boid.edges_behavior(100.0f, 700.0f, 500.0f, 100.0f, 0.2f);
  }
}

int main()
{
  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
  window.setFramerateLimit(60);

  std::vector<pf::Boid> boids;
  for (int i = 0; i < 30; ++i) {
    pf::Vector pos(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);
    pf::Vector vel((rand() % 20 - 10) * 0.1f, (rand() % 20 - 10) * 0.1f);
    boids.emplace_back(pos, vel);
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    window.clear(sf::Color::Black);

    // Update boids
    update_boids(boids, 40.0f, 0.05f, 0.005f);

    // Draw boids
    for (auto& boid : boids) {
      boid.draw(window);
    }

    window.display();
  }

  return 0;
}