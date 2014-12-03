/**
* IronDomeApp.cpp
* ---------------
* Implementation of the IronDomeApp class.
*/

#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <sutil/CSystemClock.hpp>

#include "ostreamlock.hpp"
#include "IronDomeApp.hpp"

using namespace std;

static const double PI = 3.14159265;

static const double SIMULATION_DT = 0.0001;
static const double GRAPHICS_DT = 0.020;

// Position of the operational point relative
// to the end-effector
static const Eigen::Vector3d OP_POS(0, 0.0, 6.5*2.54/100);

// Robot states
static const int STATE_UNINIT = -1;
static const int STATE_IDLE = 0;
static const int STATE_TARGETING = 1;
static const int STATE_PAUSED = 2;

// Choose one robot
#define IIWA
//#define KUKA
//#define PUMA

#ifdef IIWA
static const string robot_name("iiwaBot");
static const string graphics_name("iiwaBotStdView");
static const string config_file("./specs/iiwa/iiwaCfg.xml");

static const Eigen::Vector3d READY_POSITION(0.524, 0, 1.034);
static const Eigen::Quaterniond READY_ORIENTATION(0.92, 0, 0.38, 0);

static const Eigen::Vector3d START_POSITION(0.6, 0, 0.58);
//static const Eigen::Quaterniond START_ORIENTATION(.01, 1, 0, 0);

Eigen::Matrix3d START_ROTATION;

// Our sphere of interest for interceptions
//static const Eigen::Vector3d COLLISION_SPHERE_POS(0, 0, 0.5);
//static const double COLLISION_SPHERE_RADIUS = 0.75;
static const Eigen::Vector3d COLLISION_SPHERE_POS(-.4, 0, 0.1);
static const double COLLISION_SPHERE_RADIUS = 1.35;
#endif

#ifdef KUKA
static const string robot_name("KukaBot");
static const string graphics_name("KukaBotStdView");
static const string config_file("./specs/Kuka/KukaCfg.xml");

static const Eigen::Vector3d START_POSITION(0, 0, 0);
static const Eigen::Quaterniond START_ORIENTATION(1, 0, 1, 0);

// Our sphere of interest for interceptions
static const Eigen::Vector3d COLLISION_SPHERE_POS(0, 0, -0.54);
static const double COLLISION_SPHERE_RADIUS = 0.7;
#endif

#ifdef PUMA
static const string robot_name("PumaBot");
static const string graphics_name("PumaBotStdView");
static const string config_file("./specs/Puma/PumaCfg.xml");

static const Eigen::Vector3d START_POSITION(0, 0, 0.9);
static const Eigen::Quaterniond START_ORIENTATION(1, 0, 1, 0);

// Our sphere of interest for interceptions
static const Eigen::Vector3d COLLISION_SPHERE_POS(0, 0, 0.338);
static const double COLLISION_SPHERE_RADIUS = 0.8;
#endif

static const string VISION_ENDPOINT = "tcp://localhost:4242";
//static const string VISION_ENDPOINT = "tcp://192.168.150.2:4242";
static const string ROBOT_PORT = "tcp://*:3883";
static const string ROBOT_ENDPOINT = "tcp://localhost:4244";

static const double KP_P = 15000;
static const double KV_P = 1000;
static const double KP_R = 11000;
static const double KV_R = 500;

static const double KP_Q_BASE = 30;
static const double KV_Q_BASE = 8;

// How far towards the desired position to command
static const double DX_MAX_MAGNITUDE = 0.13;
static const double DPHI_MAX_MAGNITUDE = 0.25;

// Constraints on which targets to intercept
static const double T_INTERCEPT_MIN = 0.3;
static const double Z_INTERCEPT_MIN = 0.65;
static const double X_INTERCEPT_MIN = 0.35;
static const double Y_INTERCEPT_WIDTH = 0.45;

static const double JOINT_LIMIT_EPSILON = 3 * PI / 180;

static const vector<double> K_JLIM = {100, 150, 100, 100, 100, 100, 100};

static const bool gravityCompEnabled = true;

IronDomeApp::IronDomeApp() : t(0), t_sim(0), iter(0), finished(false),
        op_pos(OP_POS), kp_p(KP_P), kv_p(KV_P), kp_r(KP_R), kv_r(KV_R),
        state(STATE_UNINIT), paused(true), simulation(true), joint_space(false) {

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

  q_sensor = rio.sensors_.q_;
  q_d = rio.sensors_.q_;

  cout << fixed << setprecision(3);

  dof = rio.dof_;

  tau_jlim = Eigen::VectorXd(dof);

  // TODO read in these from XML file?
  kp_q = Eigen::VectorXd(dof);
  kv_q = Eigen::VectorXd(dof);

  Eigen::VectorXd kp_q_scale(dof);
  for(int i = 0; i < dof; i++) {
    double torque_range = rds.rb_tree_.at(i)->force_gc_lim_upper_
        - rds.rb_tree_.at(i)->force_gc_lim_lower_;
    kp_q(i) = KP_Q_BASE * torque_range;
    kv_q(i) = KV_Q_BASE * torque_range;
  }

  ee = rgcm.rbdyn_tree_.at("end-effector");

  ready_pos_joint = Eigen::VectorXd(dof);
  ready_pos_joint << 0, 1.5, 0, -1.6, 0, .85, 0;

  START_ROTATION << -1, 0, 0, 0, 1, 0, 0, 0, -1;

  setDesiredPosition(START_POSITION);
  setDesiredOrientation(START_ROTATION);

  // Start the clock
  sutil::CSystemClock::start();

  state = STATE_IDLE;

  cout << oslock << "Initialized IronDomeApp for " << robot_name
       << " with " << dof << " degrees of freedom." << endl << osunlock;
}

void IronDomeApp::translate(double x, double y, double z) {
  Eigen::Vector3d pos(x, y, z);
  lock_guard<mutex> lg(data_lock);
  x_d += pos;
}

void IronDomeApp::rotate(double x, double y, double z) {

  Eigen::Matrix3d temp = R_d * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
  lock_guard<mutex> lg(data_lock);
  R_d = temp;
}

void IronDomeApp::setDesiredJointPosition(const Eigen::VectorXd& q_new) {
  lock_guard<mutex> lg(data_lock);
  q_d = q_new;
}

void IronDomeApp::setDesiredPosition(double x, double y, double z) {
  lock_guard<mutex> lg(data_lock);
  x_d << x, y, z;
}

void IronDomeApp::setDesiredPosition(const Eigen::Vector3d& pos) {
  lock_guard<mutex> lg(data_lock);
  x_d = pos;
}

void IronDomeApp::setDesiredOrientation(const Eigen::Matrix3d& R) {
  lock_guard<mutex> lg(data_lock);
  R_d = R;
}

void IronDomeApp::setDesiredOrientation(const Eigen::Quaterniond& quat) {
  Eigen::Matrix3d temp = quat.normalized().toRotationMatrix();
  lock_guard<mutex> lg(data_lock);
  R_d = temp;
}

void IronDomeApp::setDesiredOrientation(double x, double y, double z) {
  auto temp = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
  lock_guard<mutex> lg(data_lock);
  R_d = temp;
}

void IronDomeApp::setControlGains(double kp_p, double kv_p, double kp_r, double kv_r) {
  lock_guard<mutex> lg(data_lock);
  this->kp_p = kp_p;
  this->kv_p = kv_p;
  this->kp_r = kp_r;
  this->kv_r = kv_r;
}

void IronDomeApp::setJointFrictionDamping(double kv_friction) {
  lock_guard<mutex> lg(data_lock);
  for(int i = 0; i < dof; i++) {
    rds.rb_tree_.at(i)->friction_gc_kv_ = kv_friction * 1.3;
  }
}

void IronDomeApp::updateState() {

  lock_guard<mutex> lg(data_lock);

  double t_new = sutil::CSystemClock::getSysTime();
  double t_sim_new = sutil::CSystemClock::getSimTime();
  dt_sim = (t_sim_new - t_sim);
  dt_real = (t_new - t);
  if(iter%10000 == 0)
    cout << oslock << "dt real: " << dt_real*1000
        << " dt sim: " << dt_sim*1000 << endl << osunlock;
  t = t_new;
  t_sim = t_sim_new;

  // Update joint angles either from integration or from robot encoders
  if(simulation) {
    q = rio.sensors_.q_;
  } else {
    q = rio.sensors_.q_;
    //q = q_sensor;
    //rio.sensors_.q_ = q;
  }

//  // TODO
//  for (int i=0; i < dof; i++){
//      if (!testJointLimit(i)){
//          double qmax = rds.gc_pos_limit_max_[i];
//          double qmin = rds.gc_pos_limit_min_[i];
//          if (abs(q[i] - qmax) < abs(q[i] - qmin))
//               q[i] = qmax - JOINT_LIMIT_EPSILON - 0.001;
//          else
//               q[i] = qmin + JOINT_LIMIT_EPSILON + 0.001;
//        rio.sensors_.dq_[i] = 0;
//        rio.sensors_.ddq_[i] = 0;
//      }
//  }

  // Just set these from the integration
  // TODO maybe estimate from q, if they don't match up
  dq = rio.sensors_.dq_;
  ddq = rio.sensors_.ddq_;

  // Compute kinematic quantities
  dyn_scl.computeGCModel(&rio.sensors_, &rgcm);
  //dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_, q);
  dyn_scl.computeJacobianWithTransforms(J, *ee, q, op_pos);

  lambda_inv = J * rgcm.M_gc_inv_ * J.transpose();
  lambda = lambda_inv.inverse();

  g_q = rgcm.force_gc_grav_;

  x_c = ee->T_o_lnk_ * op_pos;
  R_c = ee->T_o_lnk_.rotation();

  v = J.block(0, 0, 3, dof) * dq;
  omega = J.block(3, 0, 3, dof) * dq;
}

void IronDomeApp::commandTorque(Eigen::VectorXd torque) {
  lock_guard<mutex> lg(data_lock);
  rio.actuators_.force_gc_commanded_ = torque;
}

bool IronDomeApp::isPaused() {
  lock_guard<mutex> lg(data_lock);
  return paused;
}

void IronDomeApp::stateMachine() {

  //cout << oslock << "State: " << state << endl << osunlock;

  // Pause if needed
  if(isPaused() && (state != STATE_PAUSED)) {
    state = STATE_PAUSED;
    target = NULL;
    setDesiredPosition(START_POSITION);
    setDesiredOrientation(START_ROTATION);
  }

  projectile_manager.updateActiveProjectiles();
  auto active_projectiles = projectile_manager.getActiveProjectiles();

  if(state == STATE_PAUSED) {

    if(!isPaused())
      state = STATE_IDLE;

  } if(state == STATE_IDLE) {

    Projectile* best_target = NULL;
    for(pair<const int, Projectile*>& p : active_projectiles) {

      Projectile* proj = p.second;
      if(!best_target) {

        double tIntersect = proj->getIntersectionTime(
            COLLISION_SPHERE_POS,
            COLLISION_SPHERE_RADIUS
        );

        if(tIntersect >= T_INTERCEPT_MIN) {
          Eigen::Vector3d collision_pos = proj->getPosition(tIntersect);
          if((collision_pos[2] > Z_INTERCEPT_MIN) &&
             (abs(collision_pos[1]) < Y_INTERCEPT_WIDTH) &&
                 (collision_pos[0] > X_INTERCEPT_MIN)) {
            best_target = proj;
          }
        }
      } else {
        // TODO compare proj to best_target
        // if better target, best_target = proj
        continue;
      }
    }

    if(best_target) {
      target = best_target;
      state = STATE_TARGETING;
      cout << oslock << "Now targeting projectile " << target->getID() << endl << osunlock;
    } else {
      state = STATE_IDLE;
    }

    data_lock.lock();
    joint_space = true;
    data_lock.unlock();

    setDesiredJointPosition(ready_pos_joint);
    setDesiredPosition(READY_POSITION);
//    setDesiredOrientation(READY_ORIENTATION);

  } else if(state == STATE_TARGETING) {

    data_lock.lock();
    joint_space = false;
    data_lock.unlock();

    if(active_projectiles.find(target->getID()) == active_projectiles.end()) {
      target = NULL;
      state = STATE_IDLE;
      return;
    }

    double tIntersect = target->getIntersectionTime(
        COLLISION_SPHERE_POS,
        COLLISION_SPHERE_RADIUS
    );

    if(tIntersect >= 0) {
      Eigen::Vector3d collision_pos = target->getPosition(tIntersect);
      setDesiredPosition(collision_pos);

      Eigen::Vector3d desired_z_axis = -target->getVelocity(tIntersect);
      desired_z_axis.normalize();

      setDesiredOrientation(
          Eigen::Quaterniond::FromTwoVectors(
              Eigen::Vector3d::UnitZ(), desired_z_axis
          )
      );
    }
  } else if(state == STATE_UNINIT) {
    throw std::runtime_error("Uninitialized!");
  }
}

void IronDomeApp::fullTaskSpaceControl() {

  lock_guard<mutex> lg(data_lock);

  // Position error vector
  dx = x_c - x_d;

  // Calculate the position force
  F_p = -kp_p * dx - kv_p * v;

  // Rotational error vector
  dphi = -0.5 * (
      R_c.col(0).cross(R_d.col(0))
          + R_c.col(1).cross(R_d.col(1))
          + R_c.col(2).cross(R_d.col(2))
  );

  // Calculate the rotation force
  F_r = -kp_r * dphi - kv_r * omega;

  // Superimpose the forces
  F << F_p, F_r;

  tau = J.transpose() * (lambda * F);
};

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void IronDomeApp::incrementalTaskSpaceControl() {

  lock_guard<mutex> lg(data_lock);

  // Position error vector
  dx = x_c - x_d;

  // Clamp the position error vector
  double dist_p = min(dx.norm(), DX_MAX_MAGNITUDE);
  dx = dx.normalized() * dist_p;

  // Calculate the position force
  F_p = -kp_p * dx - kv_p * v;

  // Rotational error vector
  dphi = -0.5 * (
      R_c.col(0).cross(R_d.col(0))
    + R_c.col(1).cross(R_d.col(1))
    + R_c.col(2).cross(R_d.col(2))
  );

  // Clamp the rotation error vector
  double dist_r = min(dphi.norm(), DPHI_MAX_MAGNITUDE);
  dphi = dphi.normalized() * dist_r;

  // Calculate the rotation force
  F_r = -kp_r * dphi - kv_r * omega;

  // Superimpose the forces
  F << F_p, F_r;

  tau = J.transpose() * (lambda * F);
};

void IronDomeApp::resolvedMotionRateControl() {

  lock_guard<mutex> lg(data_lock);

  // Position error vector
  dx = x_c - x_d;

  // Clamp the position error vector
  double dist_p = min(dx.norm(), DX_MAX_MAGNITUDE);
  dx = dx.normalized() * dist_p;

  // Rotational error vector
  dphi = -0.5 * (
      R_c.col(0).cross(R_d.col(0))
          + R_c.col(1).cross(R_d.col(1))
          + R_c.col(2).cross(R_d.col(2))
  );

  // Clamp the rotation error vector
  double dist_r = min(dphi.norm(), DPHI_MAX_MAGNITUDE);
  dphi = dphi.normalized() * dist_r;

  dphi *= 0.3;

  // Combined rate in task space
  Eigen::VectorXd v_task(7);
  v_task << dx, dphi;

  double lambda = 0.1;

  Eigen::JacobiSVD<Eigen::MatrixXd> J_svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto sv = J_svd.singularValues();
  Eigen::MatrixXd J_opt(6, dof);
  const Eigen::MatrixXd& U = J_svd.matrixU();
  const Eigen::MatrixXd& V = J_svd.matrixV();

  for(int i = 0; i < sv.size(); i++) {
    J_opt += (sv(i) / (sv(i) * sv(i) + lambda * lambda)) * V.col(i) * U.col(i).transpose();
  }

  // Combined rate in joint space
  Eigen::VectorXd v_joint = J.transpose() * v_task;

  // Gains
  double kp_q_base = 4000;
  Eigen::VectorXd kp_q_scale(dof);
  for(int i = 0; i < dof; i++) {
    double torque_range = rds.rb_tree_.at(i)->force_gc_lim_upper_
        - rds.rb_tree_.at(i)->force_gc_lim_lower_;
    kp_q(i) = kp_q_base * torque_range;
    kv_q(i) = sqrt(kp_q(i)) * 1;
  }

//  cout << oslock << "kp_q: " << kp_q.transpose() <<
//      "\nkv_q: " << kv_q.transpose() << endl << osunlock;

  tau = - kp_q.array() * v_joint.array() - kv_q.array() * dq.array();

//  cout << oslock <<
//      "v_task: " << v_task.transpose() << "\nv_joint: " << v_joint.transpose()
//      << "\ntau: " << tau.transpose()
//      << endl << osunlock;
}

void IronDomeApp::jointSpaceControl() {

  lock_guard<mutex> lg(data_lock);

  // Joint error vector
  q_diff = q - q_d;

  Eigen::VectorXd F = -kp_q.array() * q_diff.array() - kv_q.array() * dq.array();
  tau = rgcm.M_gc_ * F;
}

void IronDomeApp::applyGravityCompensation() {
  tau += g_q;
}

void IronDomeApp::applyTorqueLimits() {
  for(int i = 0; i < dof; i++) {
    tau(i) = min(tau(i), rds.rb_tree_.at(i)->force_gc_lim_upper_*2.0);
    tau(i) = max(tau(i), rds.rb_tree_.at(i)->force_gc_lim_lower_*2.0);
  }
}

void IronDomeApp::applyJointFriction() {
  for(int i = 0; i < dof; i++) {
    tau(i) += -rds.rb_tree_.at(i)->friction_gc_kv_ * dq(i);
  }
}

void IronDomeApp::applyJointLimitPotential() {

  Eigen::VectorXd q_0 = (rds.gc_pos_limit_max_ + rds.gc_pos_limit_min_) / 2;

  Eigen::VectorXd q_range = rds.gc_pos_limit_max_ - q_0;

  q_sat = (q - q_0).array() / q_range.array();

  for(int i = 0; i < dof; i++) {
    if(q_sat[i] >= +1) q_sat[i] = +0.98;
    if(q_sat[i] <= -1) q_sat[i] = -0.98;
    tau_jlim[i] = - K_JLIM[i] * tan(PI/2 * q_sat[i]*q_sat[i]*q_sat[i]);
  }
  tau += tau_jlim;
  //cout << oslock << "Restoring torque: " << restoring_torque.transpose() << endl << osunlock;
}

void IronDomeApp::integrate() {

  lock_guard<mutex> lg(data_lock);
  dyn_tao.integrate(rio, SIMULATION_DT);
  iter++;
}

void IronDomeApp::sendToRobot(zmqpp::socket& robot_pub) {

  data_lock.lock();

  // Position error vector
  dx = x_c - x_d;

  // Clamp the position error vector
  double dist_p = min(dx.norm(), DX_MAX_MAGNITUDE);
  dx = dx.normalized() * dist_p;

  // Incremental position towards goal
  x_inc = x_c - dx;

  // Scale from m to mm
  x_inc *= 1000;

  zmqpp::message msg;
  msg << to_string(q[0]) + " " +
             to_string(q[1]) + " " +
             to_string(q[2]) + " " +
             to_string(-q[3]) + " " +
             to_string(q[4]) + " " +
             to_string(q[5]) + " " +
             to_string(0); //rio.sensors_.q_[6]);
//  msg << to_string(x_inc[0]) + " " +
//      to_string(x_inc[1]) + " " +
//      to_string(x_inc[2]) + " " +
//      to_string(R_d(0, 0)) + " " +
//      to_string(R_d(1, 0)) + " " +
//      to_string(R_d(2, 0)) + " " +
//      to_string(R_d(0, 1)) + " " +
//      to_string(R_d(1, 1)) + " " +
//      to_string(R_d(2, 1)) + " " +
//      to_string(R_d(0, 2)) + " " +
//      to_string(R_d(1, 2)) + " " +
//      to_string(R_d(2, 2));

  data_lock.unlock();

  robot_pub.send(msg);
}

void IronDomeApp::controlsLoop() {

  // Initialize a 0MQ publisher socket
  zmqpp::context context;
  zmqpp::socket robot_pub(context, zmqpp::socket_type::publish);

  robot_pub.bind(ROBOT_PORT);

  while(!finished) {

    data_lock.lock();
    bool simulation_enabled = simulation;
    bool joint_space_enabled = joint_space;
    data_lock.unlock();

    updateState();

    if(!simulation_enabled) sendToRobot(robot_pub);

    stateMachine();

    if(!joint_space_enabled) {
      // Compute the ideal joint torques based on some algorithm
      //fullTaskSpaceControl();
      incrementalTaskSpaceControl();
      //resolvedMotionRateControl();
    } else {
      jointSpaceControl();
    }

    // Add gravity compensation in joint space
    if(gravityCompEnabled) applyGravityCompensation();

    // Apply forces to keep away from joint limits
    applyJointLimitPotential();

    // Simulate joint friction
    applyJointFriction();

    // Clamp the commanded torques
    applyTorqueLimits();

    commandTorque(tau);
    integrate();

    double t_new = sutil::CSystemClock::getSysTime();
    double t_wait = SIMULATION_DT - (t_new - t);
//    if(iter % 950 == 0) {
//      cout << oslock << "t_wait: " << t_wait*1000 << endl << osunlock;
//    }
    //t_wait = 0;//.00001;
    if(t_wait > 0) t_wait = 0;

    long nanosec = static_cast<long>(t_wait * 1e9);
    const timespec ts = {0, nanosec};
    nanosleep(&ts, NULL);

    sutil::CSystemClock::tick(SIMULATION_DT);
  }
}

void IronDomeApp::graphicsLoop() {

  // Current position
  chai3d::cMaterial x_c_mat;
  x_c_mat.setBlueMediumSlate();
  chai3d::cMesh x_c_sphere(&x_c_mat);
  chai3d::cCreateSphere(&x_c_sphere, 0.02);
  chai_world->addChild(&x_c_sphere);

  // Desired position
  chai3d::cMaterial x_d_mat;
  x_d_mat.setBlue();
  chai3d::cMesh x_d_sphere(&x_d_mat);
  x_d_sphere.setUseTransparency(true);
  x_d_sphere.setTransparencyLevel(0.5);
  chai3d::cCreateSphere(&x_d_sphere, 0.03);
  chai_world->addChild(&x_d_sphere);

  // Collision sphere
  chai3d::cMaterial collision_sphere_mat;
  collision_sphere_mat.setColorf(1, 1, 1, 0.2);
  collision_sphere_mat.setTransparencyLevel(0.5);
  chai3d::cMesh collision_sphere(&collision_sphere_mat);
  collision_sphere.setUseTransparency(true);
  collision_sphere.setTransparencyLevel(0.1);
  chai3d::cCreateSphere(&collision_sphere, COLLISION_SPHERE_RADIUS);
  chai_world->addChild(&collision_sphere);
  collision_sphere.setLocalPos(
      COLLISION_SPHERE_POS(0),
      COLLISION_SPHERE_POS(1),
      COLLISION_SPHERE_POS(2)
  );

  // Projectiles
  map<int, chai3d::cMesh> projectile_spheres;
  map<int, chai3d::cMesh> projectile_spheres_m;
  map<int, chai3d::cMesh> projectile_spheres_c;
  chai3d::cMaterial projectile_mat;
  projectile_mat.setYellow();
  chai3d::cMaterial projectile_mat_m;
  projectile_mat_m.setGreen();
  chai3d::cMaterial projectile_mat_c;
  projectile_mat_c.setGray();

  long nanosec = static_cast<long>(GRAPHICS_DT * 1e9);
  const timespec ts = {0, nanosec};
  while(!finished) {

    // Draw control points
    x_c_sphere.setLocalPos(x_c[0], x_c[1], x_c[2]);
    x_d_sphere.setLocalPos(x_d[0], x_d[1], x_d[2]);

    auto active_projectiles = projectile_manager.getActiveProjectiles();

    // Draw projectiles
    for(pair<const int, Projectile*>& p : active_projectiles) {
      Projectile* proj = p.second;
      int id = proj->getID();

      // Create a new sphere if needed
      if(projectile_spheres.find(id) == projectile_spheres.end()) {

        projectile_spheres[id] = chai3d::cMesh(&projectile_mat);
        chai3d::cCreateSphere(&projectile_spheres[id], 0.04);
        chai_world->addChild(&projectile_spheres[id]);

        projectile_spheres_m[id] = chai3d::cMesh(&projectile_mat_m);
        chai3d::cCreateSphere(&projectile_spheres_m[id], 0.04);
        chai_world->addChild(&projectile_spheres_m[id]);

        projectile_spheres_c[id] = chai3d::cMesh(&projectile_mat_c);
        chai3d::cCreateSphere(&projectile_spheres_c[id], 0.04);
        chai_world->addChild(&projectile_spheres_c[id]);
      }

      // Set sphere position
      double now = sutil::CSystemClock::getSysTime();
      Eigen::Vector3d pos = proj->getPosition(now);
      projectile_spheres[id].setLocalPos(pos(0), pos(1), pos(2));
      const Eigen::Vector3d& pObs = proj->getLastObservedPosition();
      projectile_spheres_m[id].setLocalPos(pObs(0), pObs(1), pObs(2));

      double tIntersect = proj->getIntersectionTime(COLLISION_SPHERE_POS, COLLISION_SPHERE_RADIUS);
      if(tIntersect >= 0) {
        Eigen::Vector3d collision_pos = proj->getPosition(tIntersect);
        projectile_spheres_c[id].setLocalPos(collision_pos(0), collision_pos(1), collision_pos(2));
//        cout << oslock << "Projectile " << proj.id << " will collide with sphere at t = "
//             << tIntersect << " and position = " << collision_pos.transpose() << endl << osunlock;
      }
    }

    // Remove spheres that are no longer representing active projectiles
    for(auto it = projectile_spheres.begin(); it != projectile_spheres.end();) {
      if(active_projectiles.find((*it).first) == active_projectiles.end()) {
        chai_world->removeChild(&(*it).second);
        projectile_spheres.erase(it++);
      } else {
        ++it;
      }
    }

    for(auto it = projectile_spheres_m.begin(); it != projectile_spheres_m.end();) {
      if(active_projectiles.find((*it).first) == active_projectiles.end()) {
        chai_world->removeChild(&(*it).second);
        projectile_spheres_m.erase(it++);
      } else {
        ++it;
      }
    }

    for(auto it = projectile_spheres_c.begin(); it != projectile_spheres_c.end();) {
      if(active_projectiles.find((*it).first) == active_projectiles.end()) {
        chai_world->removeChild(&(*it).second);
        projectile_spheres_c.erase(it++);
      } else {
        ++it;
      }
    }

    glutMainLoopEvent();
    nanosleep(&ts, NULL);

    if(!scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      finished = true;
  }
}

void printHelp() {
  cout << oslock
      << "Commands:\n"
      << "  [p]rint                            Print matrices for debugging.\n"
      << "  [h]elp                             Print this help message.\n"
      << "  [a]ctivate                         Start projectile interception.\n"
      << "  [d]eactivate                       Stop projectile interception.\n"
      << "  [e]xit                             Exit the simulation.\n"
      << "  [m]ove x, y, z                     Move to by vector (x, y, z).\n"
      << "  [t]ranslate x, y, z                Translate by vector (x, y, z).\n"
      << "  [r]otate x, y, z                   Rotate by XYZ euler angles.\n"
      << "  [o]rientation x, y, z              Orient to XYZ euler angles.\n"
      << "  [q]uaternion w, x, y, z            Orient to quaternion (w, x, y, z).\n"
      << "  [g]ains kp_p, kv_p, kp_r, kv_r     Set the task-space control gains.\n"
      << "  [f]riction kv_friction             Set the joint friction gain.\n"
      << "  [s]witch                           Switch between simulation and robot.\n"
      << "  [j]oint                            Toggle joint space control.\n"
      << "  jmo[v]e                            Command a position in joint space.\n"
      << endl << osunlock;
}

void IronDomeApp::printState() {

  cout << oslock;

  cout << "======= Frame " << iter << " =======\n";
  cout << "t = " << t << ", t_sim = " << t_sim << "\n";
  cout << "dt_sim = " << dt_sim << ", " << "dt_real = " << dt_real << "\n\n";

  cout << "M_gc = \n" << rgcm.M_gc_ << "\n\n";
  //cout << "lambda_inv = \n" << lambda_inv << "\n\n";
  cout << "lambda = \n" << lambda << "\n\n";
  cout << "J = \n" << J << "\n\n";

  cout << "  F = " << F.transpose() << "\n";
  cout << "tau = " << rio.actuators_.force_gc_commanded_.transpose() << "\n";
  cout << "tau_jlim = " << tau_jlim.transpose() << "\n\n";

  cout << "  q = " <<   q.transpose() << "\n"
       << " dq = " <<  dq.transpose() << "\n"
       << "ddq = " << ddq.transpose() << "\n"
       << "q_sat = " << q_sat.transpose() << "\n\n";

  cout << " x_c = " << x_c.transpose() << "\n"
       << " x_d = " << x_d.transpose() << "\n"
       << " x_inc = " << x_inc.transpose() << "\n"
       << " R_c = \n" << R_c << "\n";

  cout << osunlock;
}

void IronDomeApp::visionLoop() {

  // Initialize a 0MQ subscribe socket
  zmqpp::context context;
  zmqpp::socket socket(context, zmqpp::socket_type::subscribe);
  socket.subscribe("");

  // Connect to the endpoint providing vision data
  socket.connect(VISION_ENDPOINT);

  while(!finished) {

    // Receive a message
    zmqpp::message message;
    socket.receive(message);

    int id; // Unique ID number of projectile
    double time, x, y, z; // Timestamp and measured position

    string msg;
    message >> msg;
    stringstream msg_stream(msg);
    cout << "Message: " << oslock << msg << endl << osunlock;

    // Read the message into the variables
    msg_stream >> id >> time >> x >> y >> z;

    if(msg_stream.fail()) {
      cerr << oslock << "ERROR: Invalid projectile measurement received: "
           << msg << endl << osunlock;
      continue;
    }

    projectile_manager.addObservation(id, time, x, y, z);
  }
}

void IronDomeApp::shellLoop() {

  // Pause so initial printout is after all threads launch.
  long nanosec = static_cast<long>(0.1 * 1e9);
  const timespec ts = {0, nanosec};
  nanosleep(&ts, NULL);

  cout << oslock << "\n"
      << "*******************************\n"
      << "* Iron Dome Interactive Shell *\n"
      << "*******************************\n"
      << endl << osunlock;

  printHelp();

  while(!finished) {
    cout << oslock << ">> " << osunlock;

    string cmd;
    cin >> cmd;

    if((cmd == "move") || (cmd == "m")) {

      double x, y, z;
      cin >> x >> y >> z;
      cout << oslock << "Moving to "
          << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      setDesiredPosition(x, y, z);

    } else if((cmd == "translate") || (cmd == "t")) {

      double x, y, z;
      cin >> x >> y >> z;
      cout << oslock << "Translating by "
           << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      translate(x, y, z);

    } else if((cmd == "rotate") || (cmd == "r")) {

      double x, y, z;
      cin >> x >> y >> z;
      cout << oslock << "Rotating by XYZ euler angles "
           << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      rotate(x, y, z);

    } else if((cmd == "orientation") || (cmd == "o")) {

      double x, y, z;
      cin >> x >> y >> z;
      cout << oslock << "Orienting to XYZ euler angles "
          << "(" << x << ", " << y << ", " << z << ")" << endl << osunlock;
      setDesiredOrientation(x, y, z);

    } else if((cmd == "quaterion") || (cmd == "q")) {

      double w, x, y, z;
      cin >> w >> x >> y >> z;
      cout << oslock << "Orienting to quaternion "
          << "(" << w << ", " << x << ", " << y << ", " << z << ")" << endl << osunlock;
      Eigen::Quaterniond quat(w, x, y, z);
      setDesiredOrientation(quat);

    } else if((cmd == "gains") || (cmd == "g")) {

      double kp_p, kv_p, kp_r, kv_r;
      cin >> kp_p >> kv_p >> kp_r >> kv_r;
      cout << oslock << "Setting control gains "
           << "kp_p = " << kp_p << ", kp_v = " << kv_p
           << ", kp_r = " << kp_r << ", kv_r = " << kv_r
           << endl << osunlock;
      setControlGains(kp_p, kv_p, kp_r, kv_r);

    } else if((cmd == "friction") || (cmd == "f")) {

      double kv_friction;
      cin >> kv_friction;
      cout << oslock << "Setting joint friction kv_friction = " << kv_friction
           << endl << osunlock;
      setJointFrictionDamping(kv_friction);

    } else if((cmd == "activate") || (cmd == "a")) {
      data_lock.lock();
      paused = false;
      data_lock.unlock();
      cout << oslock << "Starting projecticle defense." << endl << osunlock;

    } else if((cmd == "deactivate") || (cmd == "d")) {
      data_lock.lock();
      paused = true;
      data_lock.unlock();
      cout << oslock << "Stopping projecticle defense." << endl << osunlock;

    } else if((cmd == "switch") || (cmd == "s")) {
      data_lock.lock();
      if(simulation) {
        simulation = false;
        q_sensor = q;
        cout << oslock << "Now controlling physical robot!!" << endl << osunlock;
      } else {
        simulation = true;
        cout << oslock << "Now simulating." << endl << osunlock;
      }
      data_lock.unlock();

    } else if((cmd == "joint") || (cmd == "j")) {
      data_lock.lock();
      if(joint_space) {
        joint_space = false;
        x_d = x_c;
        cout << oslock << "Now in task-space control." << endl << osunlock;
      } else {
        joint_space = true;
        q_d = q;
        cout << oslock << "Now in joint-space control." << endl << osunlock;
      }
      data_lock.unlock();

    } else if((cmd == "jmove") || (cmd == "v")) {
      double q0, q1, q2, q3, q4, q5, q6;
      cin >> q0 >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
      Eigen::VectorXd q_new(dof);
      q_new << q0, q1, q2, q3, q4, q5, q6;
      cout << oslock << "Orienting to joint position "
          << q_new.transpose() << endl << osunlock;
      setDesiredJointPosition(q_new);

    } else if((cmd == "print") || (cmd == "p")) {
      printState();

    } else if((cmd == "help") || (cmd == "h")) {
      printHelp();

    } else if((cmd == "exit") || (cmd == "e")) {
      finished = true;

    } else {
      cout << oslock << "Command not understood!" << endl << osunlock;
      printHelp();

      // Skip rest of the line
      cin.ignore(10000, '\n');
    }

    // Clear the error flag
    cin.clear();
  }
}

void IronDomeApp::robotLoop(){

  zmqpp::context context;
  zmqpp::socket socket_sub (context, zmqpp::socket_type::subscribe);

  socket_sub.connect(ROBOT_ENDPOINT);
  socket_sub.subscribe("");

  while(!finished){

    zmqpp::message message_sub;
    socket_sub.receive(message_sub);
    string msg;
    message_sub >> msg;
    stringstream msg_stream(msg);

    // Read the message into the variables
    //cout << oslock << msg << endl << osunlock;
    data_lock.lock();
    msg_stream >> q_sensor[0] >> q_sensor[1] >> q_sensor[2] >> q_sensor[3]
               >> q_sensor[4] >> q_sensor[5] >> q_sensor[6];
    q_sensor[3] = -q_sensor[3];
    data_lock.unlock();
  }
}

bool IronDomeApp::testJointLimit(int joint_num){
   if ( q[joint_num] < rds.gc_pos_limit_max_[joint_num] - JOINT_LIMIT_EPSILON &&
        q[joint_num] > rds.gc_pos_limit_min_[joint_num] + JOINT_LIMIT_EPSILON)
   {
     return true;
   }
   else return false;
}
