/**
*
*/

#include "CIronDomeApp.hpp"
#include "IronDomeCallbacks.hpp"

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#endif
#include <util/HelperFunctions.hpp>

#include <sutil/CSystemClock.hpp>

#include <omp.h>
#include <GL/freeglut.h>

using namespace scl;
using namespace scl_app;

int main(int argc, char** argv) {

  //Set the cwd and specs dir so scl knows where the graphics are.
  scl_util::getCurrentDir(CDatabase::getData()->cwd_);
  CDatabase::getData()->dir_specs_ = CDatabase::getData()->cwd_ + std::string("specs/");

  //Initialize glut before the app
#ifdef GRAPHICS_ON
  if(!CDatabase::getData()->s_gui_.glut_initialized_) {
    glutInit(&argc, argv);
    CDatabase::getData()->s_gui_.glut_initialized_ = true;
  }
#endif

  //Convert the argc and arv into a vector of strings that the app can read
  //If you have custom paths etc, please set them here.
  std::vector<std::string> argvec;
  for(int i=0;i<argc;++i) {
    argvec.push_back(std::string(argv[i]));
  }

  //Initialize the app
  scl_app::CIronDomeApp app;
  if(!app.init(argvec)) {
    return 1;
  }

  //Register all the callback functions.
  if(!registerCallbacks()) {
    std::cout<<"\nFailed to register callbacks"; return 1;
  }

  //Set up the threads
#ifndef DEBUG
  omp_set_num_threads(3);
#else
  omp_set_num_threads(1);
#endif

  int thread_id;
  if(!app.setInitialStateForUIAndDynamics()) {
    return 1;
  }

  app.t_start_ = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
  {//Start threaded region
    thread_id = omp_get_thread_num();

    /***********************Main Loop*****************************/
    if(thread_id == 2) { app.runConsoleShell(); }
    else {//No shell in debug mode for now.
#ifndef DEBUG
      app.runMainLoopThreaded(thread_id);  //Run multi-threaded in release mode
#else
      app.runMainLoop();          //Run single-threaded in debug mode
#endif
    }
  }
  app.t_end_ = sutil::CSystemClock::getSysTime();
  std::cout << "\nSimulation Took Time : " << app.t_end_-app.t_start_ << " sec";

  /****************************Deallocate Memory And Exit*****************************/
  app.terminate();
  return 0;
}
