/**
*
*/

#pragma once

#include <map>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include "kalman.hpp"

/**
* One data point for a projectile.
*/
class ProjectileMeasurement {
public:
  double t; // Measured time
  double x, y, z; // Measured position

  ProjectileMeasurement(double t, double x, double y, double z) :
      t(t), x(x), y(y), z(z) {}
};

/**
* Projectile class.
*/
class Projectile {

public:

  Projectile(int id, const ProjectileMeasurement& m0);
  Projectile() {};

  /**
  * Add a measured value to the estimator.
  */
  void addObservation(const ProjectileMeasurement& obs);

  /**
  * Read the projectile's estimated state at time t.
  */
  Eigen::Vector3d getPosition(double t) const;
  Eigen::Vector3d getVelocity(double t) const;
  Eigen::Vector3d getAcceleration(double t) const;

  // ID number
  int id;

  // Offset between this program's time and the reporting
  // program's time (t0 - tObs0)
  double tOffset;

  // Time of last estimate
  double t;

  // Estimated state, with pos/vel/acc vectors of x, y, z
  // TODO watch out for thread-unsafe access here
  Eigen::Vector3d p, v, a;

  // Last measurement
  Eigen::Vector3d pObs;

private:

  const static int n = 3; // Number of states
  const static int m = 1; // Number of measurements


  // Estimated state, with x, y, z vectors of pos/vel/acc
  Eigen::Vector3d x, y, z;

  // State estimators
  KalmanFilter estimatorX, estimatorY, estimatorZ;
};

// ----------------------------

/**
* Projectile manager class.
*/
class ProjectileManager {

public:

  ProjectileManager();

  /**
  * Update active projectiles.
  */
  void update();

  /**
  * Add a measurement to the given projectile ID, or register
  * a new one if this is the first observation.
  */
  void addObservation(int id, double t, double x, double y, double z);

  /**
  * Return a const reference to the active projectiles.
  */
  const std::map<int, Projectile>& getActiveProjectiles();

private:

  // List of active projectiles
  std::map<int, Projectile> projectiles;
};
