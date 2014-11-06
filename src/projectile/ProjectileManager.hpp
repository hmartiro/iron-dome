/**
*
*/

#pragma once

#include <map>
#include <string>
#include <fstream>

#include "ProjectileGenerator.hpp"

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
  * Close down the output file.
  */
  void close();

  /**
  * Call init(), then loop through update() until finished
  * is set to true. Convenience method only.
  */
  void run();

  /**
  * Return a const reference to the active projectiles.
  */
  const std::map<int, Projectile>& getActiveProjectiles();

private:

  // Class used to generate projectiles in simulation
  ProjectileGenerator& pg;

  // List of active projectiles
  std::map<int, Projectile> projectiles;

  // Next projectile to be generated
  Projectile pendingProjectile;
  bool pendingProjectileExists;

  // Stop run()
  bool finished;

  // File to write data to
  std::string outfile_name;
  std::ofstream outfile;
};
