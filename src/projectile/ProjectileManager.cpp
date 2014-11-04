/**
*
*/

#include "ProjectileManager.hpp"

#include <sutil/CSystemClock.hpp>
#include <iostream>
#include <algorithm>

ProjectileManager::ProjectileManager(ProjectileGenerator& pg) :
  pg(pg) {}

void ProjectileManager::init() {

  finished = false;
  pendingProjectileExists = false;
  projectiles = {};
}

void ProjectileManager::update() {

  double now = sutil::CSystemClock::getSysTime();

  // Queue up the next pending projectile
  if(!pendingProjectileExists) {
    pendingProjectile = pg.getNextProjectile();
    pendingProjectileExists = true;
  }

  // Activate the pending projectile when the time comes
  if(now >= pendingProjectile.t0) {
    projectiles.push_back(pendingProjectile);
    pendingProjectileExists = false;
  }

  // Update the position of each projectile
  for(Projectile& proj : projectiles) {
    proj.p = pg.observePosition(proj);
  }

  // Get rid of expired projectiles
  projectiles.erase(
      std::remove_if(std::begin(projectiles), std::end(projectiles), Projectile::isExpired),
      std::end(projectiles)
  );

  // Print out the current time and active projectiles
  std::cout << "(" << now << ", ";
  for(Projectile& proj : projectiles) {
    std::cout << "(" << proj.id << ", "
        << "(" << proj.p[0] << ", " << proj.p[1] << ", " << proj.p[2] << ")), ";
  }
  std::cout << ")," << std::endl;
}

void ProjectileManager::run() {

  init();
  while(!finished) {
    update();
  }
}
