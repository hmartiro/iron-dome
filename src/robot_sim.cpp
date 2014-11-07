/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file scl_tutorial4_control_gc_op.cpp
 *
 *  Created on: Nov 5, 2014
 *
 *  Copyright (C) 2014
 *
 *
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

/** A sample application to demonstrate a physics simulation in scl.
*
* Moving forward from tutorial 3, we will now control the 6 DOF
* demo robot (r6bot) with the physics engine running.
*
* SCL Modules used:
* 1. data_structs
* 2. dynamics (physics)
* 4. dynamics (control matrices)
* 4. graphics (chai)
* */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #4";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...
  sutil::CSystemClock::start();

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("./specs/Puma/PumaCfg.xml","./specs/","PumaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("./specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the PumaBot physics. \nPress (x) to exit at anytime.";
  long iter = 0, n_iters=100000; double dt=0.0001;

  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;
  const Eigen::Vector3d hpos(0,0.0,0.0); //control position of op-point wrt. hand
  Eigen::MatrixXd J;
  Eigen::Vector3d x, x_des, x_des_tmp, x_init, dx;
  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSysTime();
      iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // Compute kinematic quantities
        dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
        dyn_scl.computeJacobianWithTransforms(J,*rhand,rio.sensors_.q_,hpos);

        // Position
        // -------------------------
        Eigen::MatrixXd J_p = J.block(0, 0, 3, rio.dof_);

        //Position of control point in origin frame
        Eigen::Vector3d x_c = rhand->T_o_lnk_ * hpos;
        Eigen::Vector3d x_d = {0.2, 0.4, 0.2};
        Eigen::Vector3d dx = x_c - x_d;
        Eigen::Vector3d v = J_p * rio.sensors_.dq_;

        double kp_p = 400;
        double kv_p = 40;
        Eigen::Vector3d F_p = -kp_p * dx - kv_p * v;

        Eigen::MatrixXd lambda_p_inv = (J_p * rgcm.M_gc_inv_ * J_p.transpose());
        Eigen::MatrixXd lambda_p = lambda_p_inv.inverse();

        Eigen::VectorXd tau_p = J_p.transpose() * (lambda_p * F_p);

        // Rotation
        // -------------------------
        Eigen::MatrixXd J_r = J.block(3, 0, 3, rio.dof_);

        Eigen::MatrixXd R_c = rhand->T_o_lnk_.rotation();
        Eigen::Quaterniond q_d(.707, 0, -.707, 0);
        q_d.normalize();
        Eigen::Matrix3d R_d = q_d.toRotationMatrix();

        Eigen::Vector3d s_d1 = R_d.col(0);
        Eigen::Vector3d s_d2 = R_d.col(1);
        Eigen::Vector3d s_d3 = R_d.col(2);

        Eigen::Vector3d s_c1 = R_c.col(0);
        Eigen::Vector3d s_c2 = R_c.col(1);
        Eigen::Vector3d s_c3 = R_c.col(2);

        Eigen::Vector3d dphi = -0.5 * (s_c1.cross(s_d1) + s_c2.cross(s_d2) + s_c3.cross(s_d3));
        Eigen::Vector3d omega = J_r * rio.sensors_.dq_;

        double kp_r = 2000;
        double kv_r = 400;
        Eigen::Vector3d F_r = -kp_r * dphi - kv_r * omega;

        Eigen::MatrixXd lambda_r_inv = (J_r * rgcm.M_gc_inv_ * J_r.transpose());
        Eigen::MatrixXd lambda_r = lambda_r_inv.inverse();

        Eigen::VectorXd tau_r = J_r.transpose() * (lambda_r * F_r);

        // Generalized Forces
        // -------------------------
        Eigen::VectorXd g_q = rgcm.force_gc_grav_;
        Eigen::VectorXd tau = tau_p + tau_r + g_q;
        rio.actuators_.force_gc_commanded_ = tau;

        // Integrate the dynamics
        dyn_tao.integrate(rio, dt);
        iter++;
        const timespec ts = {0, 5000};//.05ms
        nanosleep(&ts,NULL);

        //if(iter % static_cast<long>(n_iters/10) == 0)
        //{ //std::cout<<"\nTracking error: "<<(x_des-x).transpose()<<". Norm: "<<(x_des-x).norm(); }
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
