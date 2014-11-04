/**
*
*/

#pragma once

#include "ProjectileGenerator.hpp"

#include <vector>

class ProjectileManager {

public:

  ProjectileManager(ProjectileGenerator& pg);

  /**
  * Clear out all projectiles and reset data structures.
  */
  void init();

  /**
  * Generate new projectiles, update active projectiles.
  */
  void update();

  /**
  * Call init(), then loop through update() until finished
  * is set to true. Convenience method only.
  */
  void run();

private:

  // Class used to generate projectiles in simulation
  ProjectileGenerator& pg;

  // List of active projectiles
  std::vector<Projectile> projectiles;

  // Next projectile to be generated
  Projectile pendingProjectile;
  bool pendingProjectileExists;

  // Stop run()
  bool finished;
};
