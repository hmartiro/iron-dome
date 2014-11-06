/**
*
*/

#include "CIronDomeApp.hpp"
#include "IronDomeCallbacks.hpp"
#include "projectile/ProjectileGenerator.hpp"
#include "projectile/ProjectileManager.hpp"

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#endif
#include <util/HelperFunctions.hpp>

#include <sutil/CSystemClock.hpp>
#include <vector>

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
  for(int i = 0; i < argc; ++i) {
    argvec.push_back(std::string(argv[i]));
  }

  //Initialize the app
  scl_app::CIronDomeApp app;
  if(!app.init(argvec)) {
    return 1;
  }

  //Register all the callback functions.
  if(!registerCallbacks()) {
    std::cout<<"\nFailed to register callbacks";
    return 1;
  }

  if(!app.setInitialStateForUIAndDynamics())
    return 1;

  //Set up the threads
  #ifndef DEBUG
    omp_set_num_threads(4);
  #else
    omp_set_num_threads(1);
  #endif

  app.t_start_ = sutil::CSystemClock::getSysTime();

  int thread_id;
  #pragma omp parallel private(thread_id)
  {

    thread_id = omp_get_thread_num();

    std::cout << "\n Thread " << thread_id << " started!" << std::endl;

    if(thread_id == 0 || thread_id == 1) {

      #ifndef DEBUG
            app.runMainLoopThreaded(thread_id);  //Run multi-threaded in release mode
      #else
            app.runMainLoop();          //Run single-threaded in debug mode
      #endif

    } if(thread_id == 2) {

      //app.runConsoleShell();

    } else if(thread_id == 3) {

      // Projectile generation parameters
      double t_avg = 3.0;
      double v_avg = 6.5;
      double theta_avg = 45 * (M_PI / 180);

      // Test timestep and end time
      double dt = 1.0 / 30.0;

      ProjectileGenerator pg = {t_avg, v_avg, theta_avg};
      ProjectileManager pm = {pg};

      // Get graphics objects
      scl::CGraphicsChai rchai = app.chai_gr_;
      SGraphicsChai* graphics = rchai.getChaiData();
      chai3d::cWorld* chai_world = graphics->chai_world_;

      timespec ts = {0, static_cast<int>(dt * 1e9)};

      double radius = 0.04;
      std::map<int, chai3d::cMesh*> spheres;

      // Loop through the ProjectileManager
      pm.init();
      while(scl::CDatabase::getData()->running_) {

        pm.update();

        const std::map<int, Projectile>& projectiles = pm.getActiveProjectiles();
        for(const std::pair<int, Projectile>& pair: projectiles) {
          const Projectile& proj = pair.second;
          if(spheres.count(proj.id) == 0) {
            spheres[proj.id] = new chai3d::cMesh();
            chai3d::cCreateSphere(spheres[proj.id], radius);
            chai_world->addChild(spheres[proj.id]);
          }
          spheres[proj.id]->setLocalPos(proj.p[0], proj.p[1], proj.p[2]);
        }

        for(std::pair<int, chai3d::cMesh*> pair : spheres) {
          if(projectiles.count(pair.first) == 0) {
            chai_world->removeChild(pair.second);
          }
        }
        nanosleep(&ts, NULL);
      }

      pm.close();

    } else {
      std::cerr << "ERROR: Unknown thread " << thread_id << std::endl;
    }

    std::cout << "\n Thread " << thread_id << " completed!" << std::endl;
  }

  app.t_end_ = sutil::CSystemClock::getSysTime();
  std::cout << "\nSimulation Took Time : " << app.t_end_-app.t_start_ << " sec";

  app.terminate();
  return 0;
}
