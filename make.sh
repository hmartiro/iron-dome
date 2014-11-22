mkdir -p build_rel &&
cd build_rel &&
cmake ..  &&
make -j8 &&
cp -rf iron_dome ../ &&
cp -rf projectile_test ../ &&
cd ..
