#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <iostream>
#include <Eigen/src/Core/Matrix.h>

double lowestRealRoot(const Eigen::VectorXd &coeffs) {

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
    bool reRootExists = false;
    const imThreshold = 1e-10; //threshold for saying that the imaginary part is a rounding error
    const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType & r = solver.smallestRealRoot(&reRootExists,&imThreshold);
    if(reRootExists){
        return r[0];
    } else return -1;
}
