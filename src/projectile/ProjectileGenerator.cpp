/**
*
*/

#include <chrono>
#include <cmath>

#include "ProjectileGenerator.hpp"
#include <sutil/CSystemClock.hpp>

// Mean expected origin of projectiles
static const Eigen::Vector3d p0_avg = {3, 0, 0};

// Gravity vector
static const Eigen::Vector3d gravity = {0, 0, -9.81};

// Randomness of generated position and velocity
static const double position_stddev = 1.0;
static const double velocity_stddev = 0.5;

Projectile::Projectile(double t0,
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& a0) :
  t0(t0), p0(p0), v0(v0), a0(a0), p(p0), v(v0), a(a0) {}

bool Projectile::isExpired(const Projectile& proj) {
  return proj.p[0] < 0;
}

ProjectileGenerator::ProjectileGenerator(double t_avg, double v_avg, double theta_avg) :
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
    p0[i] += normal_dist(generator) * position_stddev;

  for(int i = 0; i < v0.size(); i++)
    v0[i] += normal_dist(generator) * velocity_stddev;

  // Launch time = now + exp_dist(t_avg)
  double t = sutil::CSystemClock::getSysTime() + exp_dist(generator);

  Projectile proj = {t, p0, v0, gravity};
  return proj;
}

Eigen::Vector3d ProjectileGenerator::observePosition(Projectile& proj) {

  double now = sutil::CSystemClock::getSysTime();
  double t = now - proj.t0;

  Eigen::Vector3d p = proj.p0 + proj.v0 * t + 0.5 * proj.a0 * t * t;

  return p;
}
