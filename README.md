iron-dome
=========

The Iron Dome is a projectile defense system using the Kuka LBR iiwa robot. The system dynamically identifies incoming projectiles headed toward a target, estimates their trajectories, and intercepts them in a fraction of a second.

![Iron Dome Image](website/pictures/banner.jpg?raw=true "Kuka LBR iiwa")

Our vision system uses an off-the-shelf Kinect V2 to detect and label incoming projectiles. The measurements are sent through a Kalman filter and trajectory predictor to determine the intersecting time, position and orientation with the robot defensive zone. The robot is then commanded to move to the desired configuration in advance of the projectile(s), preventing them from hitting the target. To improve robustness, stability, and reliablity of the control of the robot, we run our task space position and orientation controller in software, adding a tangent potential field to push each joint away from reaching joint limit, and clamping position and orientation errors to avoid large motions. We then feed joint angles directly from our simulated result to the physical robot, which runs a very fast joint position controller. The result is a system which can dynamically respond to thrown projectiles from a distance of as little as three meters.

Creators:

 * [Hayk Martirosyan](http://www.linkedin.com/in/hmartiro)
 * [Daniel O'Shea](https://www.linkedin.com/in/danielmoshea)
 * [Wanxi Liu](https://www.linkedin.com/in/wancy)

Videos:

 * Final Demo (video coming soon)
 * [Simulation Demo](https://www.youtube.com/watch?v=rPIiMUTwbWA)
 * [Intermediate Testing](https://www.youtube.com/watch?v=A24_O-sdZww)

**Full documentation can be found in `website/` or online [here](http://haykmartirosyan.com/project/irondome/).**

How to set up OpenNI2 and Libfreenect:
---------------------------------------

1.Download and unpack the latest version of
[libusb](http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.19/libusb-1.0.19.tar.bz2/download).
Go to the top libusb-1.0.19 folder and compile.

    ./Configure
    make
    sudo make install
    sudo ldconfig

2.Use Git clone to get [libfreenect](https://github.com/OpenKinect/libfreenect)
3.Make sure you have all dependencies installed on your computer (you can run step 4 to see if there's error)
4.Go to the top libfreenect directory and build it with the OpenNI2 driver.

    mkdir build
    cd build
    cmake .. -DBUILD_OPENNI2_DRIVER=ON
    make
    export LD_LIBRARY_PATH=($YourPathtoLibfreenectFolder)/build/lib/

5.Download and unpack OpenNI 2.2.0.33 Beta(X64/X86) from (http://structure.io/openni)

6.Go to the top OpenNI2 folder (usually OpenNI-Linux-X64-2.2) and run install

    sudo sh install.sh

7.Now copy the OpenNI2-FreenectDriver *(libFreenectDriver.so.0.5.0, libFreenectDriver.so.0.5, libFreenectDriver.so)*
    from *($YourPathtoLibfreenectFolder)/build/lib/OpenNI2-FreenectDriver* to
   *($YourPathtoOpenNI2Folder)/Samples/Bin/OpenNI2/Drivers/* and
   *($YourPathtoOpenNI2Folder)/Redist/OpenNI2/Drivers/*.

8.Plug in Kinect and in the terminal go to ($YourPathtoOpenNI2Folder)/Samples/Bin, run the SimpleViewer.

    sudo ./SimpleViewer
