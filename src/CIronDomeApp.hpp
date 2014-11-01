/**
*
*/

#pragma once

#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/CTaskComPos.hpp>

#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>

namespace scl_app {

  class CIronDomeApp : public scl::CRobotApp {

  public:
    // ****************************************************
    //                 The main functions
    // ****************************************************
    /** Runs the task controller. */
    virtual void stepMySimulation();

    // ****************************************************
    //           The initialization functions
    // ****************************************************
    /** Default constructor. Sets stuff to zero. */
    CIronDomeApp();

    /** Default destructor. Does nothing. */
    virtual ~CIronDomeApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv, scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Sets all the ui points to their current position and
     * run the dynamics once to flush the state. */
    virtual scl::sBool setInitialStateForUIAndDynamics();
  };
}
