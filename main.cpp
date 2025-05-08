#include <SFML/Graphics.hpp>
#include "boid.hpp"
#include "vector.hpp"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>

void update_boids(std::vector<pf::Boid>& boids, float separation_dist, float alignment_factor, float cohesion_factor) {
    for (auto& boid : boids) {

        std::cout << "Updating boid at position: " << boid.get_position().get_x() << ", " << boid.get_position().get_y() << std::endl;
        // Find neighboring boids within a specific distance
        std::vector<pf::Boid> neighbors = boid.Neighboring(boids, separation_dist);

        // Calculate separation, alignment, and cohesion behaviors
        pf::Vector separation = boid.Separation(neighbors, 30.0f, 3.0f);  // Example parameters
        pf::Vector alignment = boid.Alignment(neighbors, alignment_factor);
        pf::Vector cohesion = boid.Cohesion(neighbors, cohesion_factor);

        // Update the velocity by combining the behaviors
        pf::Vector delta_v = separation + alignment + cohesion;

        // Update and clamp velocity
        boid.update_velocity(delta_v);
        boid.speed_limit(3.0f);  // You can tweak this max speed

        // Update boid's position based on the new velocity
        boid.update_position(boid.get_velocity());
        pf::Vector pos = boid.get_position();

        std::cout << "New position: " << pos.get_x() << ", " << pos.get_y() << std::endl;
        // Screen wrapping behavior (boids will reappear on the other side of the window)
        if (pos.get_x() < 0) pos.set_x(800);
        if (pos.get_y() < 0) pos.set_y(600);
        if (pos.get_x() >= 800) pos.set_x(0);
        if (pos.get_y() >= 600) pos.set_y(0);
 }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
    window.setFramerateLimit(60);

    // Create some boids with random positions and velocities
    std::vector<pf::Boid> boids;
    for (int i = 0; i < 30; ++i) {
        pf::Vector pos(400 + rand() % 100 - 50, 300 + rand() % 100 - 50);  // Random positions
        pf::Vector vel((rand() % 20 - 10) * 0.1f, (rand() % 20 - 10) * 0.1f);  // Random velocities
        boids.emplace_back(pos, vel, 3.14f / 2);  // Random initial view angle
    }

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        // Update boids' positions and velocities based on flocking behavior
        update_boids(boids, 40.0f, 0.03f, 0.01f);  // You can tune these parameters

        // Draw each boid as a triangle
        for (auto& boid : boids) {
            sf::ConvexShape shape(3);  // Triangle to represent boid direction
            shape.setFillColor(sf::Color::White);
            shape.setPoint(0, sf::Vector2f(0, -10));  // Top point of the triangle
            shape.setPoint(1, sf::Vector2f(-5, 5));  // Left bottom point
            shape.setPoint(2, sf::Vector2f(5, 5));   // Right bottom point

            // Get position and set it to the shape
            pf::Vector pos = boid.get_position();
            shape.setPosition(pos.get_x(), pos.get_y());

            // Optionally rotate based on velocity direction
            pf::Vector vel = boid.get_velocity();
            float angle = std::atan2(vel.get_y(), vel.get_x()) * 180.f / 3.14159f;
            shape.setRotation(angle);

            // Draw the boid
            window.draw(shape);
        }

        window.display();
    }

    return 0;
}