/**
*
*/

#pragma once

#include <map>
#include <string>
#include <fstream>
#include <mutex>

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

//friend class ProjectileManager;

public:

  Projectile(int id, const ProjectileMeasurement& m0);
  Projectile() : id(-1) {};

  /**
  * Add a measured value to the estimator.
  */
  void addObservation(const ProjectileMeasurement& obs);

  /**
  * Read the projectile's estimated state at time t.
  */
  Eigen::Vector3d getPosition(double t);
  Eigen::Vector3d getVelocity(double t);
  Eigen::Vector3d getAcceleration(double t);

  /**
  * Get the time the projectile will first intersect with a
  * sphere at the given origin and radius, or -1 if it will
  * not happen.
  */
  double getIntersectionTime(const Eigen::Vector3d& origin, double radius);

  double getEstimateTime();
  const Eigen::Vector3d& getPositionEstimate();
  const Eigen::Vector3d& getVelocityEstimate();
  const Eigen::Vector3d& getAccelerationEstimate();
  const Eigen::Vector3d& getLastObservedPosition();

  int getID() const { return id; };

  bool isConverged();

private:

  const static int n = 3; // Number of states
  const static int m = 1; // Number of measurements

  // ID number
  int id;

  // Time of last estimate
  double t;

  // Estimated state, with pos/vel/acc vectors of x, y, z
  Eigen::Vector3d p, v, a;

  // Last measurement
  Eigen::Vector3d pObs;

  // Estimated state, with x, y, z vectors of pos/vel/acc
  Eigen::Vector3d x, y, z;

  // State estimators
  KalmanFilter estimatorX, estimatorY, estimatorZ;

  // Whether we have enough data to consider this projectile 'converged'
  bool converged;

  // How many observations we've made
  int observations;

  // Used to prevent concurrent access to variables
  std::mutex data_lock;

  // Offset between this program's time and the reporting
  // program's time (t0 - tObs0)
  double tOffset;
};

// ----------------------------

/**
* Projectile manager class.
*/
class ProjectileManager {

public:

  ProjectileManager();

  /**
  * Add a measurement to the given projectile ID, or register
  * a new one if this is the first observation.
  */
  void addObservation(int id, double t, double x, double y, double z);

  /**
  * Call periodically to handle expirations.
  */
  void updateActiveProjectiles();

  /**
  * Return a const reference to the active, converged projectiles.
  */
  std::map<int, Projectile*>& getActiveProjectiles();

private:

  // List of active projectiles
  std::map<int, Projectile*> projectiles;
  std::map<int, Projectile*> converged_projectiles;

  // Used to prevent concurrent access to projectile vectors
  std::mutex projectile_lock;
};
