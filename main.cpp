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
        pf::Vector separation = boid.Separation(neighbors, 25.0f, 1.0f);  // Example parameters
        pf::Vector alignment = boid.Alignment(neighbors, alignment_factor);
        pf::Vector cohesion = boid.Cohesion(neighbors, cohesion_factor);

        // Update the velocity by combining the behaviors
        pf::Vector new_velocity = boid.get_velocity() + separation + alignment + cohesion;

        // Update boid's position based on the new velocity
        pf::Vector new_position = boid.get_position() + new_velocity;

        std::cout << "New position: " << new_position.get_x() << ", " << new_position.get_y() << std::endl;
        // Screen wrapping behavior (boids will reappear on the other side of the window)
        if (new_position.get_x() < 0) new_position.set_x(800);
        if (new_position.get_y() < 0) new_position.set_y(600);
        if (new_position.get_x() >= 800) new_position.set_x(0);
        if (new_position.get_y() >= 600) new_position.set_y(0);

        // Update the boid with the new position and velocity
        boid = pf::Boid(new_position, new_velocity, boid.get_view_angle());
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
    window.setFramerateLimit(60);

    // Create some boids with random positions and velocities
    std::vector<pf::Boid> boids;
    for (int i = 0; i < 50; ++i) {
        pf::Vector pos(rand() % 800, rand() % 600);  // Random positions
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
        update_boids(boids, 30.0f, 1.0f, 1.0f);  // You can tune these parameters

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