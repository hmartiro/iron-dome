/**
*
*/

#include <iostream>
#include <iomanip>
#include <zmqpp/zmqpp.hpp>
#include <sutil/CSystemClock.hpp>

#include "ProjectileGenerator.hpp"

using namespace std;

int main(int argc, char* argv[]) {

  // Projectile generation parameters
  double t_avg = 3.0;
  double v_avg = 6.0;
  double theta_avg = M_PI / 4;

  // Test timestep and end time
  double dt = 1.0 / 30.0;
  double tEnd = 15;

  ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
  timespec ts = {0, static_cast<int>(dt * 1e9)};

  // Set print format
  cout << fixed << setprecision(3);

  // Loop through, generating projectiles
  double t = sutil::CSystemClock::getSysTime();
  while(t < tEnd) {

    t = sutil::CSystemClock::getSysTime();
    pg.update();

    for(const pair<int, SimProjectile>& p : pg.getProjectiles()) {
      const SimProjectile& proj = p.second;
      cout << "Projectile " << proj.id << ", t = " << t << ", position = ("
           << proj.p(0) << ", "
           << proj.p(1)  << ", "
           << proj.p(2)  << ")" << endl;
    }

    nanosleep(&ts, NULL);
  }

  return 0;
}
