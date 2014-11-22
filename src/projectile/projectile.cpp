/**
*
*/

#include <sutil/CSystemClock.hpp>
#include <iostream>
#include <ostreamlock.h>

#include "projectile.hpp"
#include "../lowestRealRoot.hpp"

static const double GRAVITY = -9.81;

// How many observations until we deem the trajectory converged
static const int CONVERGE_LIMIT = 4;

using namespace std;

// ----------------------------
// Projectile
// ----------------------------

Projectile::Projectile(int id, const ProjectileMeasurement& obs) :
    id(id), isExpired(false), converged(false), observations(0) {

  double dt = 1.0/30; // Time step

  // Time offset
  double now = sutil::CSystemClock::getSysTime();
  tOffset = now - obs.t;

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  R << 5;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

  // Construct an estimator for each axis
  estimatorX = KalmanFilter(dt, A, C, Q, R, P);
  estimatorY = KalmanFilter(dt, A, C, Q, R, P);
  estimatorZ = KalmanFilter(dt, A, C, Q, R, P);

  // Initialize using our first measurement
  t = obs.t + tOffset;
  x = Eigen::Vector3d(obs.x, 0, 0);
  y = Eigen::Vector3d(obs.y, 0, 0);
  z = Eigen::Vector3d(obs.z, 0, GRAVITY);
  estimatorX.init(t, x);
  estimatorY.init(t, y);
  estimatorZ.init(t, z);

  // Set our projectile's state
  p << x[0], y[0], z[0];
  v << x[1], y[1], z[1];
  a << x[2], y[2], z[2];

  observations += 1;
}

void Projectile::addObservation(const ProjectileMeasurement& obs) {

  // Measurement timestamp, in our system time
  double tNew = obs.t + tOffset;

  // Time that has passed since last measurement
  double dt = tNew - t;
  //cout << oslock << "Time between measurements: " << dt << endl << osunlock;

  // A matrix using this dt
  Eigen::MatrixXd A(n, n);
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;

  // Update the estimators
  Eigen::VectorXd p_x(m), p_y(m), p_z(m);
  p_x << obs.x;
  p_y << obs.y;
  p_z << obs.z;
  estimatorX.update(p_x, dt, A);
  estimatorY.update(p_y, dt, A);
  estimatorZ.update(p_z, dt, A);

  // Get the time, make sure they match up
  t = estimatorX.time();
  assert(t == estimatorY.time());
  assert(t == estimatorZ.time());

  // Read the estimated state
  x = estimatorX.state();
  y = estimatorY.state();
  z = estimatorZ.state();

  // Set our projectile's state in pos/vel/acc form
  p << x[0], y[0], z[0];
  v << x[1], y[1], z[1];
  a << x[2], y[2], z[2];

  // Save the last observation
  pObs << obs.x, obs.y, obs.z;

  observations += 1;
  if(observations >= CONVERGE_LIMIT) converged = true;
  //cout << "Observations for " << id << ": " << observations << endl;
}

Eigen::Vector3d Projectile::getPosition(double t1) const {
  return p + v*(t1-t) + 0.5 * a*(t1-t)*(t1-t);
}

Eigen::Vector3d Projectile::getVelocity(double t1) const {
  return v + a*(t1-t);
}

Eigen::Vector3d Projectile::getAcceleration(double t1) const {
  return a;
}

double Projectile::getIntersectionTime(const Eigen::Vector3d& origin, double radius) const {

  // Polynomial coefficients
  Eigen::VectorXd coeff(5);

  double x0 = p(0) - origin(0);
  double y0 = p(1) - origin(1);
  double z0 = p(2) - origin(2);
  double vx = v(0);
  double vy = v(1);
  double vz = v(2);
  double g = a(2);
  double R = radius;

  coeff[4] = g*g/4;
  coeff[3] = g*vz;
  coeff[2] = g*z0+vx*vx+vy*vy+vz*vz;
  coeff[1] = 2*(vx*x0+vy*y0+vz*z0);
  coeff[0] = x0*x0+y0*y0+z0*z0-R*R;

  double tIntersect = lowestRealRoot(coeff);

  if(tIntersect < 0) return -1;

  return tIntersect + t;
}

// ----------------------------
// Projectile Manager
// ----------------------------

ProjectileManager::ProjectileManager() {}

void ProjectileManager::addObservation(int id, double t, double x, double y, double z) {

  ProjectileMeasurement obs(t, x, y, z);

  if(projectiles.find(id) == projectiles.end()) {
    // Projectile not found, create it
    projectiles[id] = Projectile(id, obs);
  } else {
    // Add measurement for projectile
    projectiles[id].addObservation(obs);
  }

  if(projectiles[id].converged) {
    converged_projectiles[id] = projectiles[id];
  }

  // TODO handle expiration

//  cout << oslock
//       << "Updated projectile " << id << " at t = " << t << ":\n"
//       << "p = " << projectiles[id].p.transpose() << "\n"
//       << "v = " << projectiles[id].v.transpose() << "\n"
//       << "a = " << projectiles[id].a.transpose() << "\n"
//       << osunlock;
}

const std::map<int, Projectile>& ProjectileManager::getActiveProjectiles() {

  return converged_projectiles;
}
