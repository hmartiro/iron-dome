#include "STaskOpIronDome.hpp"

#include <stdexcept>
#include <iostream>

namespace scl_app {

  //0.5cm spatial resolution
#define SCL_TASKOPIRONDOME_SPATIAL_RESOLUTION 0.005
#define SCL_TASKOPIRONDOME_DOF 3

  STaskOpIronDome::STaskOpIronDome() : STaskBase(),
      link_name_(""),
      link_ds_(S_NULL),
      spatial_resolution_(SCL_TASKOPIRONDOME_SPATIAL_RESOLUTION),
      rbd_(S_NULL)
  { }

  STaskOpIronDome::~STaskOpIronDome()
  { }


  bool STaskOpIronDome::initTaskParams() {
    try {
      if(3!=dof_task_)//This is a position based op point task
      { throw(std::runtime_error("Operational point tasks MUST have 3 dofs (xyz translation at a point)."));  }

      /** Extract the extra params */
      std::string parent_link_name;
      Eigen::Vector3d pos_in_parent;

      bool contains_plink = false, contains_posinp = false;

      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = task_nonstd_params_.begin(), ite = task_nonstd_params_.end();
          it!=ite;++it)
      {
        const scl::sString2& param = *it;
        if(param.data_[0] == std::string("parent_link"))
        {
          parent_link_name = param.data_[1];
          contains_plink = true;
        }
        else if(param.data_[0] == std::string("pos_in_parent"))
        {
          std::stringstream ss(param.data_[1]);
          ss>> pos_in_parent[0];
          ss>> pos_in_parent[1];
          ss>> pos_in_parent[2];

          contains_posinp = true;
        }
      }

      //Error checks
      if(false == contains_plink)
      { throw(std::runtime_error("Task's nonstandard params do not contain a parent link name."));  }

      if(false == contains_posinp)
      { throw(std::runtime_error("Task's nonstandard params do not contain a pos in parent."));  }

      if(0>=parent_link_name.size())
      { throw(std::runtime_error("Parent link's name is too short."));  }

      link_name_ = parent_link_name;

      link_ds_ = dynamic_cast<const scl::SRigidBody *>(robot_->rb_tree_.at_const(link_name_));
      if(S_NULL == link_ds_)
      { throw(std::runtime_error("Could not find the parent link in the parsed robot data structure"));  }

      //Initalize the task data structure.
      pos_in_parent_ = pos_in_parent;

      //Set task space vector sizes stuff to zero
      x_.setZero(dof_task_);
      dx_.setZero(dof_task_);
      ddx_.setZero(dof_task_);

      x_goal_.setZero(dof_task_);
      dx_goal_.setZero(dof_task_);
      ddx_goal_.setZero(dof_task_);

      force_task_.setZero(dof_task_);
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskOpIronDome::init() : "<<e.what();
      return false;
    }
    return true;
  }
}
