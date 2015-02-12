/**
*
*/

#include <iostream>
#include <iomanip>
#include <sutil/CSystemClock.hpp>
#include "redox.hpp"

#include "ProjectileGenerator.hpp"

using namespace std;

static const string REDIS_HOST = "localhost";
static const int REDIS_PORT = 6379;

int main(int argc, char* argv[]) {

  // Projectile generation parameters
  double t_avg = 2.5;
  double v_avg = 6.5;
  double theta_avg = M_PI / 4;

  // Test timestep and end time
  double dt = 1.0 / 30.0;
  double tEnd = 1e10;

  ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
  timespec ts = {0, static_cast<int>(dt * 1e9)};

  // Set print format
  cout << fixed << setprecision(3);

  // Initialize Redis client
  redox::Redox rdx;

  // Open the connection
  cout << "Connecting to " << REDIS_HOST << ":" << REDIS_PORT << "..." << endl;
  if(!rdx.connect(REDIS_HOST, REDIS_PORT)) {
    cerr << "Could not connect to Redis server!" << endl;
    return -1;
  }

  rdx.del("iron_dome:projectiles");

  // Loop through, generating projectiles
  double t = sutil::CSystemClock::getSysTime();
  double t0 = 0;
  while(t < tEnd) {

    t = sutil::CSystemClock::getSysTime();
    pg.update();

    for(const pair<int, SimProjectile>& p : pg.getProjectiles()) {

      const SimProjectile& proj = p.second;
      stringstream message;
      message << to_string(proj.id) + " " + to_string(t-t0) + " "
               + to_string(proj.p(0)) + " "
               + to_string(proj.p(1)) + " "
               + to_string(proj.p(2));

      rdx.command({"LPUSH", "iron_dome:projectiles", message.str()});

      cout << "Projectile " << proj.id << ", t = " << t-t0 << ", position = ("
           << proj.p(0) << ", "
           << proj.p(1)  << ", "
           << proj.p(2)  << ")" << endl;
    }

    nanosleep(&ts, NULL);
  }

  return 0;
}
