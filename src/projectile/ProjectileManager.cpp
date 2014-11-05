/**
*
*/

#include "ProjectileManager.hpp"

#include <sutil/CSystemClock.hpp>
#include <iostream>
#include <algorithm>

ProjectileManager::ProjectileManager(ProjectileGenerator& pg) :
  pg(pg), outfile_name("projectile_data.out") {}

void ProjectileManager::init() {

  finished = false;
  pendingProjectileExists = false;
  projectiles = {};

  if(outfile.is_open()) outfile.close();
  outfile.open(outfile_name);
  outfile << "(\n";
}

void ProjectileManager::close() {

  outfile << ")" << std::endl;
  outfile.close();
  std::cout << "Closed.\n";
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
    projectiles[pendingProjectile.id] = pendingProjectile;
    pendingProjectileExists = false;
  }

  // Update the position of each projectile
  for(std::pair<const int, Projectile>& pair : projectiles) {
    pair.second.p = pg.observePosition(pair.second);
  }

  // Get rid of expired projectiles
  for(auto it = projectiles.begin();it != projectiles.end();) {
    if (Projectile::isExpired(*it)) {
      std::cout << "Removing expired projectile " << (*it).first << "\n";
      projectiles.erase(it++);
    } else {
      ++it;
    }
  }

  std::cout << "Time: " << now;
  // Print out the current time and active projectiles to data file
  outfile << "    (" << now << ", ";
  for(std::pair<const int, Projectile>& pair : projectiles) {
    Projectile& proj = pair.second;
    outfile << "        (" << proj.id << ", "
        << "(" << proj.p[0] << ", " << proj.p[1] << ", " << proj.p[2] << ")), ";
    std::cout << ", Projectile " << proj.id << ": " << proj.p.transpose();
  }
  outfile << ")," << std::endl;
  std::cout << std::endl;
}

void ProjectileManager::run() {

  init();
  while(!finished) {
    update();
  }
  close();
}

const std::map<int, Projectile>& ProjectileManager::getActiveProjectiles() {
  return projectiles;
}
