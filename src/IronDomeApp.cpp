/**
* IronDomeApp.cpp
* ---------------
* Implementation of the IronDomeApp class.
*/

#include <iostream>
#include <string>
#include <iomanip>

#include <sutil/CSystemClock.hpp>

#include "ostreamlock.hpp"
#include "IronDomeApp.hpp"

using namespace std;

static const double SIMULATION_DT = 0.0001;
static const double GRAPHICS_DT = 0.020;

static const Eigen::Vector3d START_POSITION(0.2, 0.2, 0.2);
static const Eigen::Quaterniond START_ORIENTATION(1, 0, 0, 0);

// Comment this line to enable KUKA vs PUMA
#define KUKA

#ifdef KUKA
static const string robot_name("KukaBot");
static const string graphics_name("KukaBotStdView");
static const string config_file("./specs/Kuka/KukaCfg.xml");
#else
static const string robot_name("PumaBot");
static const string graphics_name("PumaBotStdView");
static const string config_file("./specs/Puma/PumaCfg.xml");
#endif

static const double KP_P = 400; //400;
static const double KV_P = 40; //40
static const double KP_R = 400; //2000;
static const double KV_R = 40; //400;

static const bool gravityCompEnabled = false;

IronDomeApp::IronDomeApp() : t(0), t_sim(0), iter(0), finished(false),
        op_pos(0,0,0), kp_p(KP_P), kv_p(KV_P), kp_r(KP_R), kv_r(KV_R) {

  // Load robot spec
  bool flag = parser.readRobotFromFile(config_file,"./specs/", robot_name, rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  if(!flag) throw runtime_error("Could not initialize robot objects!");

  // Initialize graphics
  int zero = 0;
  glutInit(&zero, NULL);
  flag = parser.readGraphicsFromFile(config_file, graphics_name, rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds, &rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(!flag) throw runtime_error("Could not initialize graphics objects!");

  // Set default joint positions
  for(unsigned int i = 0; i < rds.dof_; ++i)
    rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_;

  dof = rio.dof_;

  ee = rgcm.rbdyn_tree_.at("end-effector");

  setDesiredPosition(START_POSITION);
  setDesiredOrientation(START_ORIENTATION);

  // Start the clock
  sutil::CSystemClock::start();

  cout << oslock << "Initialized IronDomeApp for " << robot_name
       << " with " << dof << " degrees of freedom." << endl << osunlock;
}

void IronDomeApp::setDesiredPosition(Eigen::Vector3d pos) {
  x_d = pos;
}

void IronDomeApp::setDesiredOrientation(Eigen::Matrix3d R) {
  R_d = R;
}

void IronDomeApp::setDesiredOrientation(Eigen::Quaterniond quat) {
  quat.normalize();
  R_d = quat.toRotationMatrix();
}

void IronDomeApp::updateState() {

  // Update sensed generalized state
  q = rio.sensors_.q_;
  dq = rio.sensors_.dq_;
  ddq = rio.sensors_.ddq_;

//  cout << oslock << "======= Frame " << iter << " =======\n" << osunlock;

//  cout << oslock << fixed << setprecision(3)
//       << "  q: " <<   q.transpose() << "\n"
//       << " dq: " <<  dq.transpose() << "\n"
//       << "ddq: " << ddq.transpose() << "\n"
//       << osunlock;

  // Compute kinematic quantities
  dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_, q);
  dyn_scl.computeJacobianWithTransforms(J, *ee, q, op_pos);

//  cout << oslock << "J: \n" << J << endl << osunlock;

  J_p = J.block(0, 0, 3, dof);
  J_r = J.block(3, 0, 3, dof);

  lambda_p_inv = (J_p * rgcm.M_gc_inv_ * J_p.transpose());
  lambda_p = lambda_p_inv.inverse();

  lambda_r_inv = (J_r * rgcm.M_gc_inv_ * J_r.transpose());
  lambda_r = lambda_r_inv.inverse();

//  cout << oslock << "M_gc: \n" << rgcm.M_gc_ << endl << osunlock;
//  cout << oslock << "M_gc_inv: \n" << rgcm.M_gc_inv_ << endl << osunlock;
//  cout << oslock << "lambda p inv: \n" << lambda_p_inv << endl << osunlock;
//  cout << oslock << "lambda r inv: \n" << lambda_r_inv << endl << osunlock;
//  cout << oslock << "lambda p: \n" << lambda_p << endl << osunlock;
//  cout << oslock << "lambda r: \n" << lambda_r << endl << osunlock;

  g_q = rgcm.force_gc_grav_;

  x_c = ee->T_o_lnk_ * op_pos;
  R_c = ee->T_o_lnk_.rotation();

  v = J_p * dq;
  omega = J_r * dq;

  double t_new = sutil::CSystemClock::getSysTime();
  double t_sim_new = sutil::CSystemClock::getSimTime();
  double dt_sim = 1000 * (t_sim_new - t_sim);
  double dt_real = 1000 * (t_new - t);

  cout << oslock << fixed << setprecision(3)
      << "Simulated dt: " << dt_sim << "ms, "
      << "Actual dt: " << dt_real << "ms\n" << osunlock;

  t = t_new;
  t_sim = t_sim_new;
}

void IronDomeApp::commandTorque(Eigen::VectorXd torque) {

//  Eigen::VectorXd tau(dof);
//  tau << 0, 0, 0.01, 0, 0, 0, 0;
  cout << oslock << "Commanded force: " << torque.transpose() << "\n" << osunlock;
  rio.actuators_.force_gc_commanded_ = torque;
}

void IronDomeApp::computeTorque() {

  // Calculate torques toward desired position
  dx = x_c - x_d;
  F_p = -kp_p * dx - kv_p * v;
  tau_p = J_p.transpose() * (lambda_p * F_p);

  // Calculate torques toward desired orientation
  dphi = -0.5 * (
      R_c.col(0).cross(R_d.col(0))
    + R_c.col(1).cross(R_d.col(1))
    + R_c.col(2).cross(R_d.col(2))
  );
  F_r = -kp_r * dphi - kv_r * omega;
  tau_r = J_r.transpose() * (lambda_r * F_r);

  // Superimpose the torques
  tau = tau_p + tau_r;

  // Add gravity compensation
  if(gravityCompEnabled) tau += g_q;
};

void IronDomeApp::integrate() {

  dyn_tao.integrate(rio, SIMULATION_DT);
  iter++;

  //if(iter == 10) exit(1);
}

void IronDomeApp::controlsLoop() {

  long nanosec = static_cast<long>(SIMULATION_DT * 1e9);
  const timespec ts = {0, nanosec};
  while(!finished) {

    updateState();
    computeTorque();
    commandTorque(tau);
    integrate();

    nanosleep(&ts, NULL);
    sutil::CSystemClock::tick(SIMULATION_DT);
  }
}

void IronDomeApp::graphicsLoop() {

  finished = !scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running;

  long nanosec = static_cast<long>(GRAPHICS_DT * 1e9);
  const timespec ts = {0, nanosec};
  while(!finished) {
    glutMainLoopEvent();
    nanosleep(&ts,NULL);
  }
}
