/**
*
*/

#include <Eigen/Dense>

#pragma once

class ProjectileEstimator {

public:

  ProjectileEstimator();

  double dt;

  Eigen::MatrixXd A;
  Eigen::MatrixXd C;


};
