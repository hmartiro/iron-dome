/**
*
*/

#include <sutil/CSystemClock.hpp>
#include <iostream>
#include <ostreamlock.h>

#include "projectile.hpp"

static const double GRAVITY = -9.81;

using namespace std;

// ----------------------------
// Projectile
// ----------------------------

Projectile::Projectile(int id, const ProjectileMeasurement& obs) : id(id) {

  double dt = 1.0/30; // Time step

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
  x = Eigen::Vector3d(obs.x, 0, 0);
  y = Eigen::Vector3d(obs.y, 0, 0);
  z = Eigen::Vector3d(obs.z, 0, GRAVITY);
  estimatorX.init(obs.t, x);
  estimatorY.init(obs.t, y);
  estimatorZ.init(obs.t, z);

  // Set our projectile's state
  p << x[0], y[0], z[0];
  v << x[1], y[1], z[1];
  a << x[2], y[2], z[2];
}

void Projectile::addObservation(const ProjectileMeasurement& obs) {

  // Update the estimators
  Eigen::VectorXd p_x(m), p_y(m), p_z(m);
  p_x << obs.x;
  p_y << obs.y;
  p_z << obs.z;
  estimatorX.update(p_x);
  estimatorY.update(p_y);
  estimatorZ.update(p_z);

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
}

Eigen::Vector3d Projectile::getPosition(double t1) {
  return p + v*(t1-t) + 0.5 * a*(t1-t)*(t1-t);
}

Eigen::Vector3d Projectile::getVelocity(double t1) {
  return v + a*(t1-t);
}

Eigen::Vector3d Projectile::getAcceleration(double t1) {
  return a;
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

  cout << oslock
       << "Updated projectile " << id << " at t = " << t << ":\n"
       << "p = " << projectiles[id].p.transpose() << "\n"
       << "v = " << projectiles[id].v.transpose() << "\n"
       << "a = " << projectiles[id].a.transpose() << "\n"
       << osunlock;
}

void ProjectileManager::update() {

  //double now = sutil::CSystemClock::getSysTime();

//  // Queue up the next pending projectile
//  if(!pendingProjectileExists) {
//    pendingProjectile = pg.getNextProjectile();
//    pendingProjectileExists = true;
//  }

//  // Activate the pending projectile when the time comes
//  if(now >= pendingProjectile.t0) {
//    projectiles[pendingProjectile.id] = pendingProjectile;
//    pendingProjectileExists = false;
//  }

//  // Update the position of each projectile
//  for(std::pair<const int, Projectile>& pair : projectiles) {
//    pair.second.p = pg.observePosition(pair.second);
//  }

//  // Get rid of expired projectiles
//  for(auto it = projectiles.begin(); it != projectiles.end();) {
//    if (Projectile::isExpired(*it)) {
//      std::cout << "Removing expired projectile " << (*it).first << "\n";
//      projectiles.erase(it++);
//    } else {
//      ++it;
//    }
//  }

}

const std::map<int, Projectile>& ProjectileManager::getActiveProjectiles() {
  return projectiles;
}
