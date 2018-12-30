#include <iostream>
#include <algorithm>
#include <vector>
#include <cstdlib>

using namespace std;

//initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                                                          float position_stdev);


int main() {

    //set standard deviation of position:
    float position_stdev = 1.0f;

    //set map horizon distance in meters 
    int map_size = 25;

    //initialize landmarks
    std::vector<float> landmark_positions {5, 10, 20};

    // initialize priors
    std::vector<float> priors = initialize_priors(map_size, landmark_positions, position_stdev);
    

    //print values to stdout 
    for (unsigned int p = 0; p < priors.size(); p++) {
        std::cout << priors[p] << endl;
    }

    return 0;

};

//TODO: Complete the initialize_priors function
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                                                          float position_stdev) {

    //initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev
    //set all priors to 0.0
    std::vector<float> priors(map_size, 0.0);
    int count_priors = 0;
    for (int i = 0; i < map_size; ++i){
        // checking for proximity to each landmark within given stdev
        for (int j = 0; j < landmark_positions.size(); ++j){
            if (abs(landmark_positions[j]-i) <= position_stdev){
                priors[i] = 1.0f;
                ++count_priors;
            }
        }
    }

    // Normalizing the priors
    for (unsigned i = 0; i < map_size; ++i)
        if (priors[i] > 0)
            priors[i] /= static_cast<float>(count_priors);

    return priors;
}

