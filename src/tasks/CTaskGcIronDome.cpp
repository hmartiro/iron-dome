/**
*
*/

#include "CTaskGcIronDome.hpp"

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>

#include <stdexcept>

namespace scl_app {
  CTaskGcIronDome::CTaskGcIronDome() :
      scl::CTaskBase(),
      data_(S_NULL)
  {}

  CTaskGcIronDome::~CTaskGcIronDome()
  {}

  bool CTaskGcIronDome::computeServo(const scl::SRobotSensors* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //TODO : COMPUTE SERVO GENERALIZED FORCES
      //data_->force_gc_ = ?
      return true;
    }
    else
    { return false; }
  }


  bool CTaskGcIronDome::computeModel(
      const scl::SRobotSensors* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //TODO : COMPUTE DYNAMIC MODEL (if not a generalized coordinate task)
      //TODO : COMPUTE NULL SPACE
      //data_->null_space_ = ?
      return true;
    }
    else
    { return false; }
  }

  scl::STaskBase* CTaskGcIronDome::getTaskData()
  { return data_; }

  bool CTaskGcIronDome::init(scl::STaskBase* arg_task_data,
      scl::CDynamicsBase* arg_dynamics)
  {
    try
    {
      if(S_NULL == arg_task_data)
      { throw(std::runtime_error("Passed a null task data structure"));  }

      if(false == arg_task_data->has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized task data structure"));  }

      if(S_NULL == arg_dynamics)
      { throw(std::runtime_error("Passed a null dynamics object"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed an uninitialized dynamics object"));  }

      data_ = dynamic_cast<STaskGcIronDome*>(arg_task_data);
      dynamics_ = arg_dynamics;

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskGcIronDome::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  void CTaskGcIronDome::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  /*******************************************
            Dynamic Type : CTaskGcIronDome

   NOTE : To enable dynamic typing for tasks, you
   must define the types for the "CTaskName" computation
   object AND the "STaskName" data structure. THIS IS NECESSARY.

   Why? So that you have a quick and easy way to specify
   custom xml parameters in the *Cfg.xml file.
   *******************************************/
  scl::sBool registerType_TaskGcEmpty()
  {
    bool flag;
    try
    {
      sutil::CDynamicType<std::string,scl_app::CTaskGcIronDome> typeCTaskGcIronDome(std::string("CTaskGcIronDome"));
      flag = typeCTaskGcIronDome.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type CTaskGcIronDome"));}

      sutil::CDynamicType<std::string,scl_app::STaskGcIronDome> typeSTaskGcIronDome(std::string("STaskGcIronDome"));
      flag = typeSTaskGcIronDome.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type STaskGcIronDome"));}

#ifdef DEBUG
      std::cout<<"\nregisterIronDomeTaskType() : Registered my cool task with the database";
#endif
    }
    catch (std::exception& e)
    {
      std::cout<<"\nregisterIronDomeTaskType() : Error : "<<e.what();
      return false;
    }
    return true;
  }
}
