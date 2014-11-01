/**
*
*/

#pragma once

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredPrintables.hpp>

#include <scl/robot/GenericCallbacks.hpp>
#include <scl/robot/GenericPrintables.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

namespace scl_app {
    /** This function registers all the console io callbacks */
    bool registerCallbacks() {

      bool flag;
      try {
        /** ************************************************************************
        * Keyboard keys for the command line shell:
        *
        * Associate keyboard keys as the key handlers for
        * ui_point_1's x, y and z values. ui_point_1 can
        * then be used in any part of the program for other
        * stuff (like controlling an operational point)
        * ************************************************************************ */
        flag = sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'w', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](0)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            's', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](0)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'a', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](1)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            'd', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](1)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'q', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](2)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            'e', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](2)) );

        flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'u', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](0)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            'j', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](0)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'h', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](1)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            'k', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](1)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
            'y', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](2)) );
        flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
            'i', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](2)) );
        if(false == flag){throw(std::runtime_error("Could not add a keyboard callback"));  }

        /** ************************************************************************
        * Add a help function to the command line shell
        * *************************************************************************/
        flag = sutil::callbacks::add<scl::CCallbackHelp, std::string, std::vector<std::string> >(
            std::string("help") );
        if(false == flag){throw(std::runtime_error("Could not add a help callback"));  }

        /** ************************************************************************
        * Add an echo function to the command line shell
        * *************************************************************************/
        flag = sutil::callbacks::add<scl::CCallbackEcho, std::string, std::vector<std::string> >(
            std::string("echo") );
        if(false == flag){throw(std::runtime_error("Could not add an echo callback"));  }

        /** ************************************************************************
        * Add a print callback. NOTE : You also need to add printables to print
        * *************************************************************************/
        flag = sutil::callbacks::add<scl::CCallbackPrint, std::string, std::vector<std::string> >(
            std::string("print") );
        if(false == flag){throw(std::runtime_error("Could not add a print callback"));  }

        flag = scl::addRobotPrintables();
        if(false == flag){throw(std::runtime_error("Could not add callbacks to print robot info"));  }

      } catch(std::exception &e) {
        std::cout<<"\nregisterCallbacks() : Error: "<<e.what();
        return false;
      }
      return true;
    }
}
