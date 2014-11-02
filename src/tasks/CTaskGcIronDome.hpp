/**
*
*/

#pragma once

#include <scl/control/task/CTaskBase.hpp>

#include "data_structs/STaskGcIronDome.hpp"

namespace scl_app {

  /** Function to register task with the database and enable
   * dynamic typing. Call this function if you want your
   * task to be initialized by specifying stuff in a file. */
  scl::sBool registerType_TaskGcEmpty();

  /** This is my cool new task. */
  class CTaskGcIronDome : public scl::CTaskBase
  {
  public:
    /*******************************************
     * CTaskBase API : TODO : IMPLEMENT THESE!!
     *******************************************/
    /** Computes the gc forces that resist gc velocity */
    virtual bool computeServo(
        const scl::SRobotSensors* arg_sensors);

    /** Does nothing. GC tasks don't require OSC models */
    virtual bool computeModel(
        const scl::SRobotSensors* arg_sensors);

    /** Return this task controller's task data structure.*/
    virtual scl::STaskBase* getTaskData();

    /** Initializes the task object. Required to set output
     * gc force dofs. */
    virtual bool init(scl::STaskBase* arg_task_data,
        scl::CDynamicsBase* arg_dynamics);

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure */
    virtual void reset();

  public:
    CTaskGcIronDome();
    virtual ~CTaskGcIronDome();

  protected:
    //This will be filled in from the file
    STaskGcIronDome* data_;

    //TODO : Add what you want.
    //myCoolDataType data2_;
  };
}