/**
*
*/

#include <chrono>
#include <cmath>
#include <iostream>

#include "ProjectileGenerator.hpp"
#include <sutil/CSystemClock.hpp>

using namespace std;

// Mean expected origin of projectiles
static const Eigen::Vector3d p0_avg = {3.4, 0, 0};

// Gravity vector
static const Eigen::Vector3d gravity = {0, 0, -9.81};

// Initial condition per-axis standard deviation
static const double p0_stddev = 0.3;
static const double v0_stddev = 0.2;

// Simulated measurement noise
static const double pObserved_stddev = 0.05;

// ------------------------------
// SimProjectile
// ------------------------------

SimProjectile::SimProjectile(int id, double t0,
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& a0) :
  id(id), t0(t0), p0(p0), v0(v0), a0(a0) {}

bool SimProjectile::isExpired() {
  return p[0] < 0.5;
}

// ------------------------------
// ProjectileGenerator
// ------------------------------

ProjectileGenerator::ProjectileGenerator(double t_avg, double v_avg, double theta_avg) :
  count(0),
  t_avg(t_avg),
  v_avg(v_avg),
  theta_avg(theta_avg),
  exp_dist(1.0/t_avg),
  normal_dist(0.0, 1.0),
  pendingProjectileExists(false) {

    // Initialize our random number generator with a time-based seed
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
  }

SimProjectile ProjectileGenerator::getNextProjectile() {

  Eigen::Vector3d p0 = p0_avg;
  Eigen::Vector3d v0 = {-v_avg*cos(theta_avg), 0, v_avg*sin(theta_avg)};

  for(int i = 0; i < p0.size(); i++)
    p0[i] += normal_dist(generator) * p0_stddev;

  for(int i = 0; i < v0.size(); i++)
    v0[i] += normal_dist(generator) * v0_stddev;

  // Launch time = now + exp_dist(t_avg)
  double t = sutil::CSystemClock::getSysTime() + exp_dist(generator);

  count++;

  SimProjectile proj = {count, t, p0, v0, gravity};
  return proj;
}

Eigen::Vector3d ProjectileGenerator::observePosition(const SimProjectile& proj) {

  double now = sutil::CSystemClock::getSysTime();
  double t = now - proj.t0;

  // Calculate the exact position based on initial parameters
  Eigen::Vector3d p = proj.p0 + proj.v0 * t + 0.5 * proj.a0 * t * t;

  // Add some gaussian noise
  for(int i = 0; i < p.size(); i++)
    p[i] += normal_dist(generator) * pObserved_stddev;

  return p;
}

void ProjectileGenerator::update() {

  // Queue up the next pending projectile
  if(!pendingProjectileExists) {
    pendingProjectile = getNextProjectile();
    pendingProjectileExists = true;
  }

  // Activate the pending projectile when the time comes
  double now = sutil::CSystemClock::getSysTime();
  if(now >= pendingProjectile.t0) {
    projectiles[pendingProjectile.id] = pendingProjectile;
    pendingProjectileExists = false;
    cout << "Created projectile " << pendingProjectile.id << " at t = " << now << "\n"
         << "    p0 = " << pendingProjectile.p0.transpose() << "\n"
         << "    v0 = " << pendingProjectile.v0.transpose() << "\n"
         << "    a0 = " << pendingProjectile.a0.transpose() << "\n";
  }

  // Update the position of each projectile
  for(std::pair<const int, SimProjectile>& p : projectiles) {
    p.second.p = observePosition(p.second);
  }

  // Get rid of expired projectiles
  // Special method of iteration because we are deleting
  for(auto it = projectiles.begin(); it != projectiles.end();) {
    if ((*it).second.isExpired()) {
      std::cout << "Removing expired projectile " << (*it).first << "\n";
      projectiles.erase(it++);
    } else {
      ++it;
    }
  }
}

const std::map<int, SimProjectile>& ProjectileGenerator::getProjectiles() {
  return projectiles;
}
