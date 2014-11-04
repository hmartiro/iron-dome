/**
*
*/

#include <iostream>

#include "ProjectileManager.hpp"
#include "ProjectileEstimator.hpp"
#include "ProjectileGenerator.hpp"

int main(int argc, char* argv[]) {

  double t_avg = 3.0;
  double v_avg = 6.0;
  double theta_avg = M_PI / 4;

  ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
  Projectile p = pg.getNextProjectile();

  std::cout << "New projectile in " << p.t << "s from position " << p.p0.transpose()
      << " with velocity " << p.v0.transpose() << ".\n";

  ProjectileManager pm = {pg};
  pm.init();
  for(int i = 0; i < 10; i++) {
    pm.update();
    sleep(1);
  }

  return 0;
}
