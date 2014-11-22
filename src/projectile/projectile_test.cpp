/**
*
*/

#include <iostream>
#include <iomanip>
#include <zmqpp/zmqpp.hpp>
#include <sutil/CSystemClock.hpp>

#include "ProjectileGenerator.hpp"

using namespace std;

const string ADDRESS = "tcp://*:4242";

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

  // Initialize a 0MQ publish socket
  zmqpp::context context;
  zmqpp::socket socket (context, zmqpp::socket_type::publish);

  // Open the connection
  cout << "Binding to " << ADDRESS << "..." << endl;
  socket.bind(ADDRESS);

  // Loop through, generating projectiles
  double t = sutil::CSystemClock::getSysTime();
  while(t < tEnd) {

    t = sutil::CSystemClock::getSysTime();
    pg.update();

    for(const pair<int, SimProjectile>& p : pg.getProjectiles()) {

      const SimProjectile& proj = p.second;
      zmqpp::message message;
//      stringstream ss;
//      ss << " hello: ";// << proj.p.transpose();
//      cout << "Message: " << ss << endl;
//              << proj.p(0) << " "
//              << proj.p(1) << " "
//              << proj.p(2);
      message << to_string(proj.id) + " " + to_string(t) + " "
               + to_string(proj.p(0)) + " "
               + to_string(proj.p(1)) + " "
               + to_string(proj.p(2));
      
      socket.send(message);

      cout << "Projectile " << proj.id << ", t = " << t << ", position = ("
           << proj.p(0) << ", "
           << proj.p(1)  << ", "
           << proj.p(2)  << ")" << endl;
    }

    nanosleep(&ts, NULL);
  }

  return 0;
}
