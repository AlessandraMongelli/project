#include "boid.hpp"
#include "vector.hpp"
#include "flock.hpp"
#include <cmath>

namespace pf{

Flock::Flock() {
    flock_; 
};

Flock::Flock(std::vector<Boid> flock) {
    flock_ = flock;
};

Boid Flock::get_boids() const {
    for (int i = 0; i < flock_.size(); i++)
    {
        return flock_[i]; 
    }   
};


}
 