/**
*
*/

#include "ProjectileManager.hpp"

#include <sutil/CSystemClock.hpp>
#include <iostream>

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

  // Make the pending projectile active when the time comes
  if(now >= pendingProjectile.t) {
    projectiles.push_back(pendingProjectile);
    pendingProjectileExists = false;
  }

  // Print out the positon of each projectile
  for(Projectile& p : projectiles) {
    std::cout << "Projectile is at position " << p.p0.transpose()
        << " with velocity " << p.v0.transpose() << "\n";
  }
}

void ProjectileManager::run() {

  init();
  while(!finished) {
    update();
  }
}
