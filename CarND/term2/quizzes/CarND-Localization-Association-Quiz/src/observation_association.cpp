#include <cmath>
#include <iostream>
#include <vector>
#include "helpers.h"

using std::vector;

int main() {
  // define the particle
  Particle p;
  p.id = 1;
  p.x = 4;
  p.y = 5;
  p.theta = -M_PI/2; // -90 degrees
  p.weight = 1.0;
    
  // define our observations - {id, x, y}
  int num_obs = 3;
  Position obs1, obs2, obs3;
  obs1 = {1, 2, 2};
  obs2 = {2, 3, -2};
  obs3 = {3, 0, -4};
    
  // define our landmarks - {id, x, y}
  int num_landmarks = 5;
  Position l1, l2, l3, l4, l5;
  l1 = {1, 5, 3};
  l2 = {2, 2, 1};
  l3 = {3, 6, 1};
  l4 = {4, 7, 4};
  l5 = {5, 4, 7};
    
  // create vectors to hold observations, transformations and landmarks
  vector<Position> observations(num_obs);
  vector<Position> trans_obs(num_obs);
  vector<Position> landmarks(num_landmarks);
  // Place our observations and landmarks into the related vectors
  observations = {obs1, obs2, obs3};
  landmarks = {l1, l2, l3, l4, l5};
    
  // Iterate through our three observations to transform them
  for (int i = 0; i < num_obs; ++i) {
    trans_obs[i] = transform_obs(p, observations[i]);
    // You can uncomment the below to double-check the transformation
    //std::cout << int(round(trans_obs[i].x)) << ","
    //          << int(round(trans_obs[i].y)) << std::endl;
  }
    
  /**
   * This quiz - association with nearest neighbor
   */
  // Iterate through each transformed observation to associate to a landmark
  for (int i = 0; i < num_obs; ++i) {
    int closest_landmark = 0;
    int min_dist = 999999;
    int curr_dist;
    // Iterate through all landmarks to check which is closest
    for (int j = 0; j < num_landmarks; ++j) {
      // Calculate Euclidean distance
      curr_dist = sqrt(pow(trans_obs[i].x - landmarks[j].x, 2)
                                          + pow(trans_obs[i].y - landmarks[j].y, 2));
      // Compare to min_dist and update if closest
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        closest_landmark = j;
      }
    }
    // Output the related association information
    std::cout << "OBS" << trans_obs[i].id << " associated to L"
              << landmarks[closest_landmark].id << std::endl;
  }
    
  return 0;
}
