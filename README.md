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
## Enabling `spidev` (Orange Pi 5)
1. Copy device tree blob file from firmware files 

`dtc -I dtb -O dts -o opi5.dts /usr/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/rk3588s-orangepi-5.dtb`

2. Edit `opi5.dts` to enable the spi4 peripheral 

Find node `spi@fecb0000` in opi5.dts and change the `status` property to "okay"

3. Turn `opi5.dts` into a `.dtb`

`dtc -I dts -o ./rk3588s-orangepi-5.dtb opi5.dts`

4. Replace the device tree blob file with the modified version

`sudo cp rk3588s-orangepi-5.dtb /usr/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/rk3588s-orangepi-5.dtb`

5. Reboot the Orange Pi 5

Now hopefully `/dev/spidev4.1` shows up. If it doesn't you might need to run `sudo modprobe spidev` to add load the module.
