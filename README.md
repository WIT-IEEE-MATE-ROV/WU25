# WU25
### 2025 MATE ROV Competition Code
## Install
1. [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. [Create a ROS 2 workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
3. cd [YOUR_WORKSPACE]
3. `source install/local_setup.bash`
4. `cd [YOUR WORKSPACE]/src`
5. `git clone https://github.com/WIT-IEEE-MATE-ROV/WU25.git --recurse-submodules`
#### Install wiringOP
6. Run `sudo ./build` in wiringOP directory
#### Install libgpiod
7. `cd libgpiod`
8. `sudo ./autogen.sh --enable-tools=yes`
9. `make && make install`
10. I forgot.
11. Profit!
## Build
1. `cd [YOUR WORKSPACE]` (Don't build inside the package)
2. `colcon build`