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
  double dt = 1.0 / 30.0;
  double tEnd = 15;

  ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
  ProjectileManager pm = {pg};
  pm.init();

  timespec ts = {0, static_cast<int>(dt * 1e9)};

  // Loop through the ProjectileManager
  pm.init();
  for(double t = 0; t < tEnd; t += dt) {
    pm.update();
    nanosleep(&ts, NULL);
  }
  pm.close();

  return 0;
}
