/**
* IronDomeApp.hpp
* ---------------
* Main class for the Iron Dome project. Holds all data structures,
* the main state machine, and computations.
*/

#pragma once

#include <mutex>
#include <Eigen/Dense>

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

#include <GL/freeglut.h>

#include "projectile/projectile.hpp"

class IronDomeApp {

public:
  IronDomeApp();

  /**
  * Loop to continuously update controls.
  * Call from a separate thread.
  */
  void controlsLoop();

  /**
  * Loop to continuously update graphics.
  * Call from a separate thread.
  */
  void graphicsLoop();

  /**
  * Loop to continuously receive projectile position measurements
  * and update trajectory estimates of them.
  * Call from a separate thread.
  */
  void visionLoop();

  /**
  * Loop to continuously get user input.
  * Call from a separate thread.
  */
  void shellLoop();

  /**
  * Command the robot to a desired state.
  */
  void setDesiredPosition(const Eigen::Vector3d& pos);
  void setDesiredPosition(double x, double y, double z);
  void setDesiredOrientation(const Eigen::Matrix3d& R);
  void setDesiredOrientation(const Eigen::Quaterniond& quat);
  void setDesiredOrientation(double x, double y, double z);

  /**
  * Relative movements of desired state.
  */
  void translate(double x, double y, double z);
  void rotate(double x, double y, double z);

  /**
  * Gains.
  */
  void setControlGains(double kp_p, double kv_p, double kp_r, double kv_r);
  void setJointFrictionDamping(double kv_friction);

  void printState();

  bool isPaused();

private:

  /**
  * Update the member variables to reflect the state of the robot.
  */
  void updateState();

  /**
  * Compute the commanded torque.
  */
  void computeTorque();

  /**
  * State machine that sets the desired position and orientation.
  */
  void stateMachine();

  void commandTorque(Eigen::VectorXd torque);

  void integrate();

  scl::SRobotParsed rds;     // Robot data structure
  scl::SGraphicsParsed rgr;  // Robot graphics data structure
  scl::SGcModel rgcm;        // Robot data structure with dynamic quantities
  scl::SRobotIO rio;         // I/O data structure
  scl::CDynamicsScl dyn_scl; // Robot kinematics and dynamics computation object
  scl::CDynamicsTao dyn_tao; // Robot physics integrator
  scl::CParserScl parser;    // Parser from file

  scl::CGraphicsChai rchai;  // Chai interface for rendering graphics
  scl::SGraphicsChai* graphics;
  chai3d::cWorld* chai_world;

  std::mutex data_lock; // Mutex that assures thread safety to data resources

  double t; // Run-time of program
  double t_sim; // Simulated time
  double dt_real, dt_sim; // Actual and simulated time between frames

  long iter; // Number of frames
  bool finished; // Flag to shut down

  scl::SRigidBodyDyn* ee; // End effector link
  const Eigen::Vector3d op_pos; // Position of operational poin w.r.t. end-effector

  int dof; // Degrees of freedom of our robot

  double kp_p, kv_p, kp_r, kv_r; // Control gains

  Eigen::MatrixXd J; // Jacobian
  Eigen::VectorXd q, dq, ddq; // Generalized position/velocity/acceleration

  Eigen::Vector3d x_c, x_d, dx; // Position, current/desired/difference
  Eigen::Vector3d v; // Linear velocity

  Eigen::Matrix3d R_c, R_d; // End-effector orientations, current/desired
  Eigen::Vector3d dphi; // Difference in end-effector orientations
  Eigen::Vector3d omega; // Angular velocity

  Eigen::Vector3d F_p, F_r;          // Task space forces, pos/rot
  Eigen::Vector6d F;
  Eigen::MatrixXd lambda, lambda_inv; // Generalized mass matrix and inverse
  Eigen::MatrixXd lambda_p_inv, lambda_p; // Generalized mass, pos/rot
  Eigen::MatrixXd lambda_r_inv, lambda_r; // Generalized inverse mass, pos/rot
  Eigen::VectorXd tau_p, tau_r; // Generalized forces, pos/rot

  Eigen::VectorXd g_q; // Generalized gravity force
  Eigen::VectorXd tau; // Commanded generalized force

  // Class for managing the current state of projectiles
  ProjectileManager projectile_manager;

  // State of the robot
  int state;

  // Projectile we are currently chasing
  Projectile* target;

  // Whether the projectile interception is paused
  bool paused;
};
