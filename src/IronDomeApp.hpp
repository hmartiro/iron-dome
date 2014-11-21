/**
* IronDomeApp.hpp
* ---------------
* Main class for the Iron Dome project. Holds all data structures,
* the main state machine, and computations.
*/

#pragma once

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
  * Command the robot to a desired state.
  */
  void setDesiredPosition(Eigen::Vector3d pos);
  void setDesiredOrientation(Eigen::Matrix3d R);
  void setDesiredOrientation(Eigen::Quaterniond quat);

private:

  /**
  * Update the member variables to reflect the state of the robot.
  */
  void updateState();

  /**
  * Compute the commanded torque.
  */
  void computeTorque();

  void commandTorque(Eigen::VectorXd torque);

  void integrate();

  scl::SRobotParsed rds;     // Robot data structure
  scl::SGraphicsParsed rgr;  // Robot graphics data structure
  scl::SGcModel rgcm;        // Robot data structure with dynamic quantities
  scl::SRobotIO rio;         // I/O data structure
  scl::CGraphicsChai rchai;  // Chai interface for rendering graphics
  scl::CDynamicsScl dyn_scl; // Robot kinematics and dynamics computation object
  scl::CDynamicsTao dyn_tao; // Robot physics integrator
  scl::CParserScl parser;    // Parser from file

  double t; // Run-time of program
  double t_sim; // Simulated time
  long iter; // Number of frames
  bool finished; // Flag to shut down

  scl::SRigidBodyDyn* ee; // End effector link
  const Eigen::Vector3d op_pos; // Position of operational poin w.r.t. end-effector

  int dof; // Degrees of freedom of our robot

  double kp_p, kv_p, kp_r, kv_r; // Control gains

  Eigen::MatrixXd J, J_p, J_r; // Jacobians, full/pos/rot
  Eigen::VectorXd q, dq, ddq; // Generalized position/velocity/acceleration

  Eigen::Vector3d x_c, x_d, dx; // Position, current/desired/difference
  Eigen::Vector3d v; // Linear velocity

  Eigen::Matrix3d R_c, R_d; // End-effector orientations, current/desired
  Eigen::Vector3d dphi; // Difference in end-effector orientations
  Eigen::Vector3d omega; // Angular velocity

  Eigen::Vector3d F_p, F_r;               // Task space forces, pos/rot
  Eigen::MatrixXd lambda_p_inv, lambda_p; // Generalized mass, pos/rot
  Eigen::MatrixXd lambda_r_inv, lambda_r; // Generalized inverse mass, pos/rot
  Eigen::VectorXd tau_p, tau_r;           // Generalized forces, pos/rot

  Eigen::VectorXd g_q; // Generalized gravity force
  Eigen::VectorXd tau; // Commanded generalized force
};
