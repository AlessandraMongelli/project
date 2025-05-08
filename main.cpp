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
  for (auto& boid : boids) {
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

    std::cout << "Updating boid at position: " << boid.get_position().get_x()
              << ", " << boid.get_position().get_y() << std::endl;
    // Find neighboring boids within a specific distance
    std::vector<pf::Boid> neighbors = boid.neighboring(boids, separation_dist);

    // Calculate separation, alignment, and cohesion behaviors
    pf::Vector separation =
        boid.separation(neighbors, 30.0f, 3.0f); // Example parameters
    pf::Vector alignment = boid.alignment(neighbors, alignment_factor);
    pf::Vector cohesion  = boid.cohesion(neighbors, cohesion_factor);

    // Update the velocity by combining the behaviors
    pf::Vector delta_v = separation + alignment + cohesion;

    // Update and clamp velocity
    boid.update_velocity(delta_v);
    boid.speed_limit(3.0f); // You can tweak this max speed

    // Update boid's position based on the new velocity
    boid.update_position(boid.get_velocity());
    pf::Vector pos = boid.get_position();

    std::cout << "New position: " << pos.get_x() << ", " << pos.get_y()
              << std::endl;
    boid.edges_behavior(800, 600);
  }
}

int main()
{
  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
  window.setFramerateLimit(60);

  // Create some boids with random positions and velocities
  std::vector<pf::Boid> boids;
  for (int i = 0; i < 30; ++i) {
    pf::Vector pos(400 + rand() % 100 - 50,
                   300 + rand() % 100 - 50); // Random positions
    pf::Vector vel((rand() % 20 - 10) * 0.1f,
                   (rand() % 20 - 10) * 0.1f); // Random velocities
    boids.emplace_back(pos, vel, 3.14f / 2);   // Random initial view angle
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    window.clear(sf::Color::Black);

    // Update boids' positions and velocities based on flocking behavior
    update_boids(boids, 40.0f, 0.03f, 0.03f); // You can tune these parameters

    // Draw each boid as a triangle
    for (auto& boid : boids) {
      sf::ConvexShape shape(3); // Triangle to represent boid direction
      shape.setFillColor(sf::Color::White);
      shape.setPoint(0, sf::Vector2f(0, -10)); // Top point of the triangle
      shape.setPoint(1, sf::Vector2f(-5, 5));  // Left bottom point
      shape.setPoint(2, sf::Vector2f(5, 5));   // Right bottom point

      // Get position and set it to the shape
      pf::Vector pos = boid.get_position();
      shape.setPosition(pos.get_x(), pos.get_y());

      // Optionally rotate based on velocity direction
      pf::Vector vel = boid.get_velocity();
      float angle    = std::atan2(vel.get_y(), vel.get_x()) * 180.f / 3.14159f;
      shape.setRotation(angle);

      // Draw the boid
      window.draw(shape);
    }

    window.display();
  }

  return 0;
}