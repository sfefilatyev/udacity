#include <cmath>
#include <iostream>

int main() {
  // define coordinates and theta
  double x_part, y_part, x_obs, y_obs, theta;
  x_part = 4;
  y_part = 5;
  x_obs = 3;
  y_obs = -2;
  theta = -M_PI/2; // -90 degrees

  // transform to map x coordinate
  double x_map;
  x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);

  // transform to map y coordinate
  double y_map;
  y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

  // (2,2)
  std::cout << int(round(x_map)) << ", " << int(round(y_map)) << std::endl;

  return 0;
}
