#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <iostream>
#include <Eigen/src/Core/Matrix.h>


double SolvePoly(Eigen::VectorXd coeffs) {

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
	const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType & r = solver.smallestRealRoot();

	for(int i =0;i<r.rows();++i) {
        std::cout << r[i] << std::endl;
    }
	return r[0];
}
