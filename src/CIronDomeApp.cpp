/**
*
*/

#include "CIronDomeApp.hpp"
#include "tasks/CTaskGcIronDome.hpp"
#include "tasks/CTaskOpIronDome.hpp"

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>

#include <sutil/CSystemClock.hpp>

#include <chai3d.h>

#include <iostream>

using namespace chai3d;

namespace scl_app {

  /** Default constructor. Sets stuff to zero. Uses a task controller*/
  CIronDomeApp::CIronDomeApp() : CRobotApp() { }

  scl::sBool CIronDomeApp::initMyController(const std::vector<std::string>& argv,
      scl::sUInt args_parsed) {

    try {
      //Ctr in array of args_parsed = (args_parsed - 1)
      //So ctr for un-parsed arg = (args_parsed - 1) + 1
      scl::sUInt args_ctr = args_parsed;

      // Check that we haven't finished parsing everything
      while(args_ctr < argv.size()) {
        /* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */
        // else if (argv[args_ctr] == "-p")
        // { }
        if(true) {
          std::cout<<"\n Possible example task options: -xxx (you can change me to suit your needs!)";
          args_ctr++;
        }
      }

      return true;
    }
    catch(std::exception &e) {
      std::cout<<"\nCIronDomeApp::initMyController() : " << e.what();
    }
    return false;
  }

  scl::sBool CIronDomeApp::registerCustomDynamicTypes() {

    bool flag;
    flag = registerType_TaskGcEmpty();
    flag = flag && registerType_TaskOpIronDome();
    return flag;
  }

  scl::sBool CIronDomeApp::setInitialStateForUIAndDynamics() {

    bool flag;
    try {
      //Compute dynamics and servo once to initialize matrices.
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();
      robot_.setGeneralizedCoordinatesToZero();
      robot_.setGeneralizedVelocitiesToZero();
      robot_.setGeneralizedAccelerationsToZero();
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();

      //Update the operational point tasks (if any)
      std::vector<SUiCtrlPointData>::iterator it,ite;
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it ) {

        if(!it->has_been_init_) {
          throw(std::runtime_error(std::string("UI Task not intialized: ")+it->name_));
        }

        if(!it->chai_pos_des_) {
          throw(std::runtime_error(std::string("UI Task's chai position vector is NULL: ")+it->name_));
        }

        if(!it->task_) {
          throw(std::runtime_error(std::string("UI Task's control object is null: ")+it->name_));
        }

        it->task_->getPos(it->pos_);

        flag = (3 == it->pos_.rows() && 1 == it->pos_.cols()) ||
            (1 == it->pos_.rows() && 3 == it->pos_.cols());
        if(!flag) {
          throw(std::runtime_error(std::string("UI task's control position vector size is incorrect: ")+it->name_));
        }

        db_->s_gui_.ui_point_[it->ui_pt_] = it->pos_;

        //Using a tmp ref to simplify code.
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));
      }

      return true;
    }
    catch(std::exception &e) {
      std::cerr<<"\nCIronDomeApp::setInitialStateForUIAndDynamics() : "<<e.what();
    }
    return false;
  }

  void CIronDomeApp::stepMySimulation() {

    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.

    //Update the operational point tasks (if any)
    std::vector<SUiCtrlPointData>::iterator it,ite;
    for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
    { it->task_->setGoalPos(db_->s_gui_.ui_point_[it->ui_pt_]); } //Set the goal position.

    //Update dynamics at a slower rate
    if(ctrl_ctr_%5 == 0) {
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
    }

    //Update graphics and/or log at a slower rate
    if(ctrl_ctr_ % 20 == 0) { // Every 2ms
      robot_.logState(true,true,true);

      //Set the positions of the ui points
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it ) {
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));

        it->task_->getPos(it->pos_);
        Eigen::VectorXd& tmp_ref2 = it->pos_;
        it->chai_pos_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
      }
    }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    /** Slow down sim to real time */
    sutil::CSystemClock::tick(scl::CDatabase::getData()->sim_dt_);
    double tcurr = sutil::CSystemClock::getSysTime();
    double tdiff = sutil::CSystemClock::getSimTime() - tcurr;
    timespec ts = {0, 0};
    if(tdiff > 0) {
      ts.tv_sec = static_cast<int>(tdiff);
      tdiff -= static_cast<int>(tdiff);
      ts.tv_nsec = tdiff*1e9;
      nanosleep(&ts,NULL);
    }

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
