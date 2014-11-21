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

static const Eigen::Vector3d START_POSITION(0.2, 0.2, -0.1);
static const Eigen::Quaterniond START_ORIENTATION(1, 0, 0, 0);

static const double JOINT_TORQUE_LIMIT = 200;

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

static const double KP_P = 1000; //400;
static const double KV_P = 150; //40
static const double KP_R = 1000; //2000;
static const double KV_R = 150; //400;


static const double KV_FRICTION = 4;

static const bool gravityCompEnabled = true;

IronDomeApp::IronDomeApp() : t(0), t_sim(0), iter(0), finished(false),
        op_pos(0,0,0), kp_p(KP_P), kv_p(KV_P), kp_r(KP_R), kv_r(KV_R), kv_friction(KV_FRICTION) {

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

  graphics = rchai.getChaiData();
  chai_world = graphics->chai_world_;

  // Set default joint positions
  for(unsigned int i = 0; i < rds.dof_; ++i)
    rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_;

  cout << fixed << setprecision(3);

  dof = rio.dof_;

  ee = rgcm.rbdyn_tree_.at("end-effector");

  setDesiredPosition(START_POSITION);
  setDesiredOrientation(START_ORIENTATION);

  // Start the clock
  sutil::CSystemClock::start();

  cout << oslock << "Initialized IronDomeApp for " << robot_name
       << " with " << dof << " degrees of freedom." << endl << osunlock;
}

void IronDomeApp::translate(double x, double y, double z) {
  Eigen::Vector3d pos(x, y, z);
  x_d += pos;
}

void IronDomeApp::rotate(double x, double y, double z) {
  R_d = R_d * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
}

void IronDomeApp::setDesiredPosition(double x, double y, double z) { x_d << x, y, z; }
void IronDomeApp::setDesiredPosition(const Eigen::Vector3d& pos) { x_d = pos; }
void IronDomeApp::setDesiredOrientation(const Eigen::Matrix3d& R) { R_d = R; }

void IronDomeApp::setDesiredOrientation(const Eigen::Quaterniond& quat) {
  R_d = quat.normalized().toRotationMatrix();
}

void IronDomeApp::setDesiredOrientation(double x, double y, double z) {
  R_d = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
}

void IronDomeApp::updateState() {

  // Update sensed generalized state
  q = rio.sensors_.q_;
  dq = rio.sensors_.dq_;
  ddq = rio.sensors_.ddq_;

  // Compute kinematic quantities
  dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_, q);
  dyn_scl.computeJacobianWithTransforms(J, *ee, q, op_pos);

  J_p = J.block(0, 0, 3, dof);
  J_r = J.block(3, 0, 3, dof);

  lambda_inv = J * rgcm.M_gc_inv_ * J.transpose();
  lambda = lambda_inv.inverse();

  g_q = rgcm.force_gc_grav_;

  x_c = ee->T_o_lnk_ * op_pos;
  R_c = ee->T_o_lnk_.rotation();

  v = J_p * dq;
  omega = J_r * dq;

  double t_new = sutil::CSystemClock::getSysTime();
  double t_sim_new = sutil::CSystemClock::getSimTime();
  dt_sim = 1000 * (t_sim_new - t_sim);
  dt_real = 1000 * (t_new - t);

  t = t_new;
  t_sim = t_sim_new;
}

void IronDomeApp::commandTorque(Eigen::VectorXd torque) {

  rio.actuators_.force_gc_commanded_ = torque;
}

void IronDomeApp::computeTorque() {

  // Calculate torques toward desired position
  dx = x_c - x_d;
  F_p = -kp_p * dx - kv_p * v;

  // Calculate torques toward desired orientation
  dphi = -0.5 * (
      R_c.col(0).cross(R_d.col(0))
    + R_c.col(1).cross(R_d.col(1))
    + R_c.col(2).cross(R_d.col(2))
  );
  F_r = -kp_r * dphi - kv_r * omega;

  // Superimpose the forces
  F << F_p, F_r;

  tau = J.transpose() * (lambda * F);
  //tau = tau_p + tau_r;

  // Add gravity compensation
  if(gravityCompEnabled) tau += g_q;

  // Add damping from friction
  tau += -kv_friction * dq;

  // Apply torque limits
  for(int i = 0; i < tau.size(); i++) {
    tau(i) = max(min(tau(i), JOINT_TORQUE_LIMIT), -JOINT_TORQUE_LIMIT);
  }
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

  chai3d::cMesh op_sphere;
  chai3d::cCreateSphere(&op_sphere, 0.04);
  chai_world->addChild(&op_sphere);

  long nanosec = static_cast<long>(GRAPHICS_DT * 1e9);
  const timespec ts = {0, nanosec};
  while(!finished) {

    op_sphere.setLocalPos(x_d[0], x_d[1], x_d[2]);

    glutMainLoopEvent();
    nanosleep(&ts, NULL);

    // finished = !scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running;
  }
}

void printHelp() {
  cout << oslock
      << "Commands:\n"
      << "  [m]ove x, y, z                Move to by vector (x, y, z).\n"
      << "  [t]ranslate x, y, z           Translate by vector (x, y, z).\n"
      << "  [r]otate x, y, z              Rotate by XYZ euler angles.\n"
      << "  [o]rientation x, y, z         Orient to XYZ euler angles.\n"
      << "  [q]uaternion w, x, y, z       Orient to quaternion (w, x, y, z).\n"
      << "  [p]rint                       Print matrices for debugging.\n"
      << "  [h]elp                        Print this help message.\n"
      << "  [e]xit                        Exit the simulation.\n"
      << endl << osunlock;
}

void IronDomeApp::printState() {

  cout << oslock;

  cout << "======= Frame " << iter << " =======\n";
  cout << "t = " << t << ", t_sim = " << t_sim << "\n";
  cout << "dt_sim = " << dt_sim << ", " << "dt_real = " << dt_real << "\n\n";

  cout << "  q = " <<   q.transpose() << "\n"
       << " dq = " <<  dq.transpose() << "\n"
       << "ddq = " << ddq.transpose() << "\n\n";

  cout << "lambda_inv = \n" << lambda_inv << "\n\n";
  cout << "lambda = \n" << lambda << "\n\n";
  cout << "J = \n" << J << "\n\n";

  cout << "  F = " << F.transpose() << "\n";
  cout << "tau = " << tau.transpose() << "\n";

  cout << osunlock;
}

void IronDomeApp::shellLoop() {

  // Pause so initial printout is after all threads launch.
  long nanosec = static_cast<long>(0.1 * 1e9);
  const timespec ts = {0, nanosec};
  nanosleep(&ts, NULL);

  cout << oslock
      << "*******************************\n"
      << "* Iron Dome Interactive Shell *\n"
      << "*******************************\n"
      << endl << osunlock;

  printHelp();

  while(!finished) {
    cout << oslock << ">> " << osunlock;

    string cmd;
    cin >> cmd;
    cout << oslock << "Command: " << cmd << endl << osunlock;
    
    if((cmd == "move") || (cmd == "m")) {

      double x, y, z;
      cin >> x;
      cin >> y;
      cin >> z;
      cout << oslock << "Moving to "
          << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      setDesiredPosition(x, y, z);

    } else if((cmd == "translate") || (cmd == "t")) {

      double x, y, z;
      cin >> x;
      cin >> y;
      cin >> z;
      cout << oslock << "Translating by "
           << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      translate(x, y, z);

    } else if((cmd == "rotate") || (cmd == "r")) {

      double x, y, z;
      cin >> x;
      cin >> y;
      cin >> z;
      cout << oslock << "Rotating by XYZ euler angles "
           << "(" << x << ", " << y << ", " << z << ")" <<endl << osunlock;
      rotate(x, y, z);

    } else if((cmd == "orientation") || (cmd == "o")) {

      double x, y, z;
      cin >> x;
      cin >> y;
      cin >> z;
      cout << oslock << "Orienting to XYZ euler angles "
          << "(" << x << ", " << y << ", " << z << ")" <<endl << osunlock;
      setDesiredOrientation(x, y, z);

    } else if((cmd == "quaterion") || (cmd == "q")) {

      double w, x, y, z;
      cin >> w;
      cin >> x;
      cin >> y;
      cin >> z;
      cout << oslock << "Orienting to quaternion "
          << "(" << w << ", " << x << ", " << y << ", " << z << ")" <<endl << osunlock;
      Eigen::Quaterniond quat(w, x, y, z);
      setDesiredOrientation(quat);

    } else if((cmd == "print") || (cmd == "p")) {
      printState();

    } else if((cmd == "help") || (cmd == "h")) {
      printHelp();

    } else if((cmd == "exit") || (cmd == "e")) {
      finished = true;

    } else if(cmd == "") {
      // Just continue on hitting enter

    } else {
      cout << oslock << "Command not understood!" << endl << osunlock;
      string trash;
      getline(cin, trash);
    }
  }
}
