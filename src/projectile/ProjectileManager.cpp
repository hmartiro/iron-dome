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
  std::cout << "Time: " << now << std::endl;

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

  // Print out the positon of each active projectile
  for(Projectile& proj : projectiles) {
    std::cout << "Projectile is at position " << proj.p.transpose()
        << " with velocity " << proj.v.transpose() << "\n";
  }
}

void ProjectileManager::run() {

  init();
  while(!finished) {
    update();
  }
}
