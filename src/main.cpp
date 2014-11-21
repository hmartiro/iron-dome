/**
*
*/

#include <iostream>
#include "ostreamlock.hpp"

#include "IronDomeApp.hpp"

using namespace std;

static const int CONTROL_THREAD = 0;
static const int GRAPHICS_THREAD = 1;
static const int VISION_THREAD = 2;
static const int SHELL_THREAD = 3;

int main(int argc, char* argv[]) {

  IronDomeApp app = {};

  omp_set_num_threads(2);
  int thread_id;

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();

    if(thread_id == CONTROL_THREAD) {

      cout << oslock << "Control thread started!" << endl << osunlock;
      app.controlsLoop();
      cout << oslock << "Control thread finished!" << endl << osunlock;

    } else if (thread_id == GRAPHICS_THREAD) {

      cout << oslock << "Graphics thread started!" << endl << osunlock;
      app.graphicsLoop();
      cout << oslock << "Graphics thread finished!" << endl << osunlock;

    }
  }

  cout << oslock << "Successfully exiting." << endl << osunlock;
  return 0;
}
