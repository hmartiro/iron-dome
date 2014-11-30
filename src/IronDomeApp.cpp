/**
* IronDomeApp.cpp
* ---------------
* Implementation of the IronDomeApp class.
*/

#include <iostream>
#include <string>
#include <iomanip>

#include <zmqpp/zmqpp.hpp>
#include <sutil/CSystemClock.hpp>

#include "ostreamlock.hpp"
#include "IronDomeApp.hpp"

using namespace std;

static const double SIMULATION_DT = 0.0001;
static const double GRAPHICS_DT = 0.020;

// Position of the operational point relative
// to the end-effector
static const Eigen::Vector3d OP_POS(0, 0.0, 0.15);

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

static const Eigen::Vector3d START_POSITION(0.2, 0, 0.95);
static const Eigen::Quaterniond START_ORIENTATION(0.92, 0, 0.38, 0);

// Our sphere of interest for interceptions
static const Eigen::Vector3d COLLISION_SPHERE_POS(0, 0, 0.5);
static const double COLLISION_SPHERE_RADIUS = 0.7;
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

static const string VISION_ENDPOINT = "tcp://192.168.1.3:4242";
static const string ROBOT_PORT = "tcp://*:3883";
static const string ROBOT_ENDPOINT = "tcp://192.168.1.2:4244";

static const double KP_P = 7000;
static const double KV_P = 500;
static const double KP_R = 5000;
static const double KV_R = 400;

// How far towards the desired position to command
static const double DX_MAX_MAGNITUDE = 0.07;
static const double DPHI_MAX_MAGNITUDE = 0.15;

// Constraints on which targets to intercept
static const double T_INTERCEPT_MIN = 0.3;
static const double HEIGHT_INTERCEPT_MIN = 0.6;

static const double JOINT_LIMIT_EPSILON = 0.1;

static const bool gravityCompEnabled = true;

IronDomeApp::IronDomeApp() : t(0), t_sim(0), iter(0), finished(false),
        op_pos(OP_POS), kp_p(KP_P), kv_p(KV_P), kp_r(KP_R), kv_r(KV_R),
        state(STATE_UNINIT), paused(true) {

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
    rds.rb_tree_.at(i)->friction_gc_kv_ = kv_friction;
  }
}

void IronDomeApp::updateState() {

  lock_guard<mutex> lg(data_lock);

  // rio.sensors_.q_ = q_sensor;

  // Update sensed generalized state
  q = rio.sensors_.q_;

  
  for (int i=0; i < dof; i++){
      if (!testJointLimit(i)){
          double qmax = rds.gc_pos_limit_max_[i];
	  double qmin = rds.gc_pos_limit_min_[i];
          if (abs(q[i] - qmax) < abs(q[i] - qmin))
               q[i] = qmax - JOINT_LIMIT_EPSILON - 0.001;
          else
               q[i] = qmin + JOINT_LIMIT_EPSILON + 0.001;
	  rio.sensors_.dq_[i] = 0;
	  rio.sensors_.ddq_[i] = 0;
      }
  }
  //rio.sensors_.q_ = q;

  dq = rio.sensors_.dq_;
  ddq = rio.sensors_.ddq_;

  // Compute kinematic quantities
  dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_, q);
  dyn_scl.computeJacobianWithTransforms(J, *ee, q, op_pos);

  lambda_inv = J * rgcm.M_gc_inv_ * J.transpose();
  lambda = lambda_inv.inverse();

  g_q = rgcm.force_gc_grav_;

  x_c = ee->T_o_lnk_ * op_pos;
  R_c = ee->T_o_lnk_.rotation();

  v = J.block(0, 0, 3, dof) * dq;
  omega = J.block(3, 0, 3, dof) * dq;

  double t_new = sutil::CSystemClock::getSysTime();
  double t_sim_new = sutil::CSystemClock::getSimTime();
  dt_sim = 1000 * (t_sim_new - t_sim);
  dt_real = 1000 * (t_new - t);

  t = t_new;
  t_sim = t_sim_new;
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
    setDesiredOrientation(START_ORIENTATION);
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
          if(collision_pos[2] > HEIGHT_INTERCEPT_MIN) {
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

    setDesiredPosition(START_POSITION);
    setDesiredOrientation(START_ORIENTATION);

  } else if(state == STATE_TARGETING) {

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

void IronDomeApp::computeTorque() {

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
  //tau = tau_p + tau_r;

/*
  for (int i=0; i < dof; i++){
    if ( !testJointLimit(i) ){
          double qmax = rds.gc_pos_limit_max_[i];
	  double qmin = rds.gc_pos_limit_min_[i];
          if (abs(q[i] - qmax) < abs(q[i] - qmin))
               tau[i] = tau[i] - 1/(qmax - q[i]) * tau[i];
          else
               tau[i] = tau[i] + 1/(qmin - q[i]) * tau[i];
    }
  }
*/

  // Add gravity compensation
  if(gravityCompEnabled) tau += g_q;

  // Apply torque limits
  for(int i = 0; i < dof; i++) {
    tau(i) = min(tau(i), rds.rb_tree_.at(i)->force_gc_lim_upper_);
    tau(i) = max(tau(i), rds.rb_tree_.at(i)->force_gc_lim_lower_);
  }

  // Add damping to simulate friction
  for(int i = 0; i < dof; i++) {
    tau(i) += -rds.rb_tree_.at(i)->friction_gc_kv_ * dq(i);
  }
};

void IronDomeApp::integrate() {

  lock_guard<mutex> lg(data_lock);
  dyn_tao.integrate(rio, SIMULATION_DT);
  iter++;
}

void IronDomeApp::controlsLoop() {

  long nanosec = static_cast<long>(SIMULATION_DT * 1e9);
  const timespec ts = {0, nanosec};

  // Initialize a 0MQ publisher socket
  zmqpp::context context_pub, context_sub;
  zmqpp::socket socket_pub (context_pub, zmqpp::socket_type::publish);
  //zmqpp::socket socket_sub (context_sub, zmqpp::socket_type::subscribe);
  
  socket_pub.bind(ROBOT_PORT);
  //socket_sub.connect(ROBOT_ENDPOINT);
  //socket_sub.subscribe("");


  while(!finished) {

    updateState();
    stateMachine();
    computeTorque();
    commandTorque(tau);
    integrate();

    zmqpp::message message_pub, message_sub;
    message_pub << to_string(rio.sensors_.q_[0]) + " " + 
               to_string(rio.sensors_.q_[1]) + " " +
               to_string(rio.sensors_.q_[2]) + " " +
               to_string(-rio.sensors_.q_[3]) + " " +
               to_string(rio.sensors_.q_[4]) + " " +
               to_string(rio.sensors_.q_[5]) + " " +
	       to_string(rio.sensors_.q_[6]);

    socket_pub.send(message_pub);
    

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
//  map<int, chai3d::cMesh> projectile_spheres_m;
  map<int, chai3d::cMesh> projectile_spheres_c;
  chai3d::cMaterial projectile_mat;
  projectile_mat.setYellow();
//  chai3d::cMaterial projectile_mat_m;
//  projectile_mat_m.setGreen();
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

//        projectile_spheres_m[id] = chai3d::cMesh(&projectile_mat_m);
//        chai3d::cCreateSphere(&projectile_spheres_m[id], 0.04);
//        chai_world->addChild(&projectile_spheres_m[id]);

        projectile_spheres_c[id] = chai3d::cMesh(&projectile_mat_c);
        chai3d::cCreateSphere(&projectile_spheres_c[id], 0.04);
        chai_world->addChild(&projectile_spheres_c[id]);
      }

      // Set sphere position
      double now = sutil::CSystemClock::getSysTime();
      Eigen::Vector3d pos = proj->getPosition(now);
      projectile_spheres[id].setLocalPos(pos(0), pos(1), pos(2));
      //const Eigen::Vector3d& pObs = proj->getLastObservedPosition();
      //projectile_spheres_m[id].setLocalPos(pObs(0), pObs(1), pObs(2));

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

//    for(auto it = projectile_spheres_m.begin(); it != projectile_spheres_m.end();) {
//      if(active_projectiles.find((*it).first) == active_projectiles.end()) {
//        chai_world->removeChild(&(*it).second);
//        projectile_spheres_m.erase(it++);
//      } else {
//        ++it;
//      }
//    }

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

  cout << "M_gc = \n" << rgcm.M_gc_ << "\n\n";
  cout << "lambda_inv = \n" << lambda_inv << "\n\n";
  cout << "lambda = \n" << lambda << "\n\n";
  cout << "J = \n" << J << "\n\n";

  cout << "  F = " << F.transpose() << "\n";
  cout << "tau = " << tau.transpose() << "\n";

  cout << osunlock;
}

void IronDomeApp::visionLoop() {

  // Initialize a 0MQ subscribe socket
  zmqpp::context context;
  zmqpp::socket socket (context, zmqpp::socket_type::subscribe);
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
  // Initialize a 0MQ publisher socket
  zmqpp::context context_sub;
  //zmqpp::socket socket_pub (context_pub, zmqpp::socket_type::publish);
  zmqpp::socket socket_sub (context_sub, zmqpp::socket_type::subscribe);
  
  //socket_pub.bind(ROBOT_PORT);
  socket_sub.connect(ROBOT_ENDPOINT);
  socket_sub.subscribe("");
  
  long nanosec = static_cast<long>(SIMULATION_DT * 1e9);
  const timespec ts = {0, nanosec};
 while(!finished){
    zmqpp::message message_sub;

   
    socket_sub.receive(message_sub);
    string msg;
    message_sub >> msg;
    stringstream msg_stream(msg);

    // Read the message into the variables
    //cout << oslock << msg << endl << osunlock;
    lock_guard<mutex> lg(data_lock);
    msg_stream >> rio.sensors_.q_[0] >> rio.sensors_.q_[1] >> rio.sensors_.q_[2] >> rio.sensors_.q_[3]
               >> rio.sensors_.q_[4] >> rio.sensors_.q_[5] >> rio.sensors_.q_[6];
    rio.sensors_.q_[3] = -rio.sensors_.q_[3];

    nanosleep(&ts, NULL);
  }
  
}

bool IronDomeApp::testJointLimit(int joint_num){
   if ( q[joint_num] < rds.gc_pos_limit_max_[joint_num] - JOINT_LIMIT_EPSILON &&
        q[joint_num] > rds.gc_pos_limit_min_[joint_num] + JOINT_LIMIT_EPSILON)
   {
	return true;
   }
   else
        return false;
}
