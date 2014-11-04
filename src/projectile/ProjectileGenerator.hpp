/**
*
*/

#pragma once

#include "Eigen/Dense"
#include <random>

class Projectile {

public:
  Projectile() {};
  Projectile(double t, Eigen::Vector3d& p0, Eigen::Vector3d& v0);
  Eigen::Vector3d p0;
  Eigen::Vector3d v0;
  double t;
};

/**
* Generates projectiles at randomized times, with randomized initial
* positions and velocities. Throwing times are based on a Poisson
* distribution, and the initial position/velocity is normally
* distributed around a mean value.
*/
class ProjectileGenerator {

public:

  /**
  * Create a generator that on average launches one projectile every
  * t_avg seconds, with a velocity magnitude v_avg and angle from
  * horizontal theta_avg. Projectiles are launched towards the robot
  * from a randomized point in the +X direction.
  */
  ProjectileGenerator(double t_avg, double v_avg, double theta_avg);

  /**
  * Return a Projectile with launch time and initial position/velocity.
  */
  Projectile getNextProjectile();

private:

  double t_avg;
  double v_avg;
  double theta_avg;

  std::default_random_engine generator;
  std::exponential_distribution<double> exp_dist;
  std::normal_distribution<double> normal_dist;
};
