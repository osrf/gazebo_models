# This model implements an imaginary ur10 / Schunk MPG 80 hybrid robot 

In addition, the world **grasp__block.world** demonstrates the robot grasping
a block while moving along a sinusoidal trajectory.

# Building the grasping and movement controller:

Build the controller from its installation directory using CMake:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Point Gazebo to the plugin directory:
    $ cd build
    $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`

Then start grasping by typing (from the build directory):

    $ gazebo ../grasp_block.world


