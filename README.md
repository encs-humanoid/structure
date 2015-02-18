structure
=========

Node and launch file organization for the IEEE ENCS Humanoid Robot Project

CAUTION: incomplete and not debugged yet

Installation (to be completed on each separate computer):

Build a catkin workspace if you haven't already:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Fetch the structure code and run the boot_node installer:

```sh
cd catkin_ws/src
git clone https://github.com/encs-humanoid/structure.git
cd structure/scripts
./install_boot_node.sh
```

  This installs the boot-time node initialization script, so that the robot
  will organize itself upon startup of all its computers.

Will run on boot, but can be manually run with this command:

```sh
sudo service ros-boot-node start
```

WARNING: For vagrant, run the above by hand after startup. The /vagrant directory
will not be available at boot time, so this is necessary for now.

Launch file organization:

Launch files are stored in catkin_ws/src/structure/ken at this time.
