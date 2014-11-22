#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <iostream>
#include <Eigen/src/Core/Matrix.h>

double lowestRealRoot(const Eigen::VectorXd &coeffs) {

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
    bool reRootExists = false;
    double imThreshold = 1e-10; //threshold for saying that the imaginary part is a rounding error
    auto r = solver.smallestRealRoot(reRootExists, imThreshold);

   if(!reRootExists) return -1;
   return r;
}
