#include <algorithm>
#include <iostream>
#include <vector>

#include "helpers.h"

using std::vector;

// set standard deviation of control:
float control_stdev = 1.0f;

// meters vehicle moves per time step
float movement_per_timestep = 1.0f;

// number of x positions on map
int map_size = 25;

// define landmarks
vector<float> landmark_positions {5, 10, 12, 20};

// declare pseudo_range_estimator function
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position);


int main() {    
  // step through each pseudo position x (i)
  for (int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);
    // get pseudo ranges
    vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, 
                                                         pseudo_position);
    // print to stdout
    if (pseudo_ranges.size() > 0) {
      for (int s = 0; s < pseudo_ranges.size(); ++s) {
        std::cout << "x: " << i << "\t" << pseudo_ranges[s] << std::endl;
      }
      std::cout << "-----------------------" << std::endl;
    }   
  } 

  return 0;
}

// TODO: Complete pseudo range estimator function
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;
            
  for(int i = 0; i < landmark_positions.size(); ++i){
      if (landmark_positions[i] - pseudo_position > 0)
        pseudo_ranges.push_back(landmark_positions[i] - pseudo_position);
  }

  //Sorting in ascending order
  sort(pseudo_ranges.begin(), pseudo_ranges.end());
    
  return pseudo_ranges;
}
