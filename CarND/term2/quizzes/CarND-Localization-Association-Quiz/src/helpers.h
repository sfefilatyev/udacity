#ifndef HELPERS_H
#define HELPERS_H

#include <cmath>

// Can hold both landmarks and landmark observations
struct Position {
  int id;
  double x;  // x-coordinate
  double y;  // y-coordinate
};

// Includes Position variables and those for Particles
struct Particle : public Position {
  double theta;  // particle orientation
  double weight; // particle weight
};

Position transform_obs(const Particle &p, const Position &obs) {
  // Transform the x and y coordinates
  double x_map, y_map;
  x_map = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
  y_map = p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y);
    
  // Create new Position to hold transformed observation
  Position transformed_obs;
  transformed_obs.id = obs.id;
  transformed_obs.x = x_map;
  transformed_obs.y = y_map;
    
  return transformed_obs;
}

#endif  // HELPERS_H
