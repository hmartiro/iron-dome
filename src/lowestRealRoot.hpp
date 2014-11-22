#include <Eigen/Dense>
/* Solve n-th order polynomial

    Pass in the coefficients with the zero-th order term first in the vector
    i.e. for a0+a1*x+a2*x^2=0 pass in [a0 a1 a2]

    example useage:
    Eigen::VectorXd coeff(5);

    double g = -9.81;
    double R = 1.4;
    double vx = 2.0;
    double vy = 2.0;
    double vz = 2.0;
    double x0 = -1.0;
    double y0 = -1.0;
    double z0 = -0.5;

    coeff[4] = g*g/4;
    coeff[3] = g*vz;
    coeff[2] = g*z0+vx*vx+vy*vy+vz*vz;
    coeff[1] = 2*(vx*x0+vy*y0+vz*z0);
    coeff[0] = x0*x0+y0*y0+z0*z0-R*R;
    SolvePoly(&coeff)
*/
double lowestRealRoot(const Eigen::VectorXd &coeffs);
