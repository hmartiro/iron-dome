iron-dome
=========

The Iron Dome is a projectile defense system using the Kuka LBR iiwa robot. The system dynamically identifies incoming projectiles headed toward a target, estimates their trajectories, and intercepts them in a fraction of a second.

![Iron Dome Image](website/pictures/banner.png?raw=true "Kuka LBR iiwa")

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
