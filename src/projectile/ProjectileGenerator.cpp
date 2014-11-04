/**
*
*/

#include <chrono>
#include <cmath>

#include "ProjectileGenerator.hpp"
#include <sutil/CSystemClock.hpp>

// Mean expected origin of projectiles
static const Eigen::Vector3d p0_avg = {3, 0, 0.5};

// Gravity vector
static const Eigen::Vector3d gravity = {0, 0, -9.81};

// Randomness parameters
static const double p0_stddev = 0.3;
static const double v0_stddev = 0.3;
static const double pObserved_stddev = 0.03;

Projectile::Projectile(int id, double t0,
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& a0) :
  id(id), t0(t0), p0(p0), v0(v0), a0(a0), p(p0), v(v0), a(a0) {}

bool Projectile::isExpired(const Projectile& proj) {
  return proj.p[0] < 0;
}

ProjectileGenerator::ProjectileGenerator(double t_avg, double v_avg, double theta_avg) :
  count(0),
  t_avg(t_avg),
  v_avg(v_avg),
  theta_avg(theta_avg),
  exp_dist(1.0/t_avg),
  normal_dist(0.0, 1.0) {

    // Initialize our random number generator with a time-based seed
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
  }

Projectile ProjectileGenerator::getNextProjectile() {

  Eigen::Vector3d p0 = p0_avg;
  Eigen::Vector3d v0 = {-v_avg*cos(theta_avg), 0, v_avg*sin(theta_avg)};

  for(int i = 0; i < p0.size(); i++)
    p0[i] += normal_dist(generator) * p0_stddev;

  for(int i = 0; i < v0.size(); i++)
    v0[i] += normal_dist(generator) * v0_stddev;

  // Launch time = now + exp_dist(t_avg)
  double t = sutil::CSystemClock::getSysTime() + exp_dist(generator);

  count++;

  Projectile proj = {count, t, p0, v0, gravity};
  return proj;
}

Eigen::Vector3d ProjectileGenerator::observePosition(Projectile& proj) {

  double now = sutil::CSystemClock::getSysTime();
  double t = now - proj.t0;

  // Calculate the exact position based on initial parameters
  Eigen::Vector3d p = proj.p0 + proj.v0 * t + 0.5 * proj.a0 * t * t;

  // Add some gaussian noise
  for(int i = 0; i < p.size(); i++)
    p[i] += normal_dist(generator) * pObserved_stddev;

  return p;
}
