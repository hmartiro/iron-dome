/**
*
*/

#include <iostream>

#include "ProjectileManager.hpp"
#include "ProjectileEstimator.hpp"
#include "ProjectileGenerator.hpp"

int main(int argc, char* argv[]) {

  // Projectile generation parameters
  double t_avg = 3.0;
  double v_avg = 6.0;
  double theta_avg = M_PI / 4;

  // Test timestep and end time
  double dt = 0.1;
  double tEnd = 10;

  ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
  ProjectileManager pm = {pg};
  pm.init();

  // Loop through the ProjectileManager
  double t = 0;
  timespec ts = {0, static_cast<int>(dt * 1e9)};
  for(t = 0; t < tEnd; t += dt) {
    pm.update();
    nanosleep(&ts, NULL);
  }

  return 0;
}
