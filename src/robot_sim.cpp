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
    bool flag = p.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/","PumaBot",rds);
    flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
    flag = flag && dyn_tao.init(rds);         //Set up integrator object
    flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
    flag = flag && rio.init(rds.name_,rds.dof_);
    for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
    if(false == flag){ return 1; }            //Error check.

    /******************************ChaiGlut Graphics************************************/
    glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

    flag = p.readGraphicsFromFile("../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
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
    Eigen::MatrixXd Jx;
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
                dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
                /*if(false == flag) {
                    x_init = rhand->T_o_lnk_ * hpos;
                    flag = true;
                }*/
                x = rhand->T_o_lnk_ * hpos;                         //Position of control point in origin frame
                dx = Jx.block(0,0,6,rio.dof_) * rio.sensors_.dq_;

                //Calculate Torques
                Eigen::Matrix3d rDes = Eigen::Matrix3d::Identity(); //Ideally, set to rotation matrix of desired rotation
                //rDes << 1,0,0,
                  //      0,1,0,
                    //    0,0,1;
                Eigen::MatrixXd rot = rhand->T_o_lnk_.rotation();
                Eigen::Vector3d rdes0 = rDes.col(0);
                Eigen::Vector3d rdes1 = rDes.col(1);
                Eigen::Vector3d rdes2 = rDes.col(2);

                Eigen::Vector3d curRot0 = rot.col(0);
                Eigen::Vector3d curRot1 = rot.col(1);
                Eigen::Vector3d curRot2 = rot.col(2);

                Eigen::Vector3d T_temp0 = rdes0.cross(curRot0);
                Eigen::Vector3d T_temp1 = rdes1.cross(curRot1);
                Eigen::Vector3d T_temp2 = rdes2.cross(curRot2);

                Eigen::Vector3d T_temp =(T_temp0+T_temp1+T_temp2);

                // Controller : fgc = Jx' (kp * (sin(t)*.1 - (x-xinit)) + kv(0 - dx)) - kqv * dq
                // Set the desired positions so that the hand draws a circle
                double sin_ampl = 0.15;
//                x_des(0) = x_init(0)+sin(tcurr-tstart)*sin_ampl;
//                x_des(1) = x_init(1)+cos(tcurr-tstart)*sin_ampl;
//                x_des(2) = 0;
                x_des_tmp(0) = 0.2;
                x_des_tmp(1) = 0;
                x_des_tmp(2) = 0;
                x_des = x_des_tmp; //SET X DESIRED HERE
                Eigen::VectorXd fstar_t = (500*(x_des-x) - 50 * dx); //add 3 torques now
                Eigen::Vector6d fstar;
                fstar << fstar_t,
                         T_temp;

                dyn_scl.computeGCModel(&(rio.sensors_),&rgcm);
                Eigen::MatrixXd Jv = Jx.block(0,0,6,rio.dof_);
                Eigen::MatrixXd lambda_inv = (Jv * rgcm.M_gc_inv_ * Jv.transpose() );
                Eigen::VectorXd f_op = lambda_inv.inverse() * fstar;
                rio.actuators_.force_gc_commanded_ = Jx.block(0,0,6,rio.dof_).transpose() * f_op - 20*rgcm.M_gc_*rio.sensors_.dq_ + rgcm.force_gc_grav_;


//                rio.actuators_.force_gc_commanded_ = Jx.block(0,0,6,rio.dof_).transpose() * (100*(x_des-x) - 20 * dx) - 20*rio.sensors_.dq_;

                // Integrate the dynamics
                dyn_tao.integrate(rio,dt);
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
