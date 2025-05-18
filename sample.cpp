#include "flock.hpp"
#include <fstream>
#include <iostream>

int main()
{
  std::cout << "simulating 6 boids \n";

  pf::Flock flock(80.0f, 10.0f, 0.4f, 0.08f, 0.004f, 150.0f, 50.0f);
  pf::Vector pos1(100.0f, 300.0f);
  pf::Vector pos2(700.0f, 300.0f);
  pf::Vector pos3(200.0f, 100.0f);
  pf::Vector pos4(400.0f, 100.0f);
  pf::Vector pos5(200.0f, 500.0f);
  pf::Vector pos6(400.0f, 500.0f);
  pf::Vector vel1(50.0f, 0.0f);
  pf::Vector vel2(-50.0f, 0.0f);
  pf::Vector vel3(50.0f, 50.0f);
  pf::Vector vel4(-50.0f, 50.0f);
  pf::Vector vel5(70.0f, -20.0f);
  pf::Vector vel6(-60.0f, -10.0f);
  pf::Boid boid1(pos1, vel1, 0);
  pf::Boid boid2(pos2, vel2, 0);
  pf::Boid boid3(pos3, vel3, 0);
  pf::Boid boid4(pos4, vel4, 0);
  pf::Boid boid5(pos5, vel5, 0);
  pf::Boid boid6(pos6, vel6, 0);
  
  std::vector<pf::Boid> boids = {boid1, boid2, boid3, boid4, boid5, boid6};
  flock.add_boids(boids);

  std::ofstream output_file("sample_data.txt");
  output_file << "Time Average Distance Standard Deviation of Distance Average "
                 "Velocity Standard Deviation of Velocity\n";
  float time_passed     = 0.0f;
  float simulation_time = 30.0f;
  while (time_passed < simulation_time) {
    float delta_t = 0.05f;
    time_passed += delta_t;
    flock.flock_update(delta_t);
    if (time_passed >= 1.0f) {
      const pf::Statistics state = flock.flock_state();
      output_file << time_passed << " " << state.average_dist << " "
                  << state.dev_dist << " " << state.average_vel << " "
                  << state.dev_vel << "\n";
    }
  }
  output_file.close();
  std::cout << "Data saved in sample_data.txt.\n";
  return 0;
}
