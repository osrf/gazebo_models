# This model implements a passive dynamic walker

The model is a hybrid of the models described in:

* Steven H. Collins, Martijn Wisse, and Andy Ruina. A three-dimensional passive-dynamic walking robot with two legs and knees. Intl. J. Robot. Res., 20(2), 2001.
* Tad McGeer. Passive dynamic walking. Intl. J. Robot. Res., 9(2):62–82, April 1990.

The walker uses the _checkerboard_ model of a planar surface with gaps
(as described by McGeer) to keep the feet from scraping as they walk along
the ground. While the ground plane is level, the gravity vector points in
a direction that will allow the machine to walk without actuation.

# Building the plugin for the walker:

The walker uses a plugin to set its initial vertical position, initial
model roll, initial link velocity, initial ''hip'' joint angle and joint
velocity, and initial pelvis velocity. You can try values other than the
defaults; these allow the machine to walk for a few iterations before falling.

The file **plugin-search.cpp** allows one to do a grid search for good 
initial values (if, for instance, using a different physics engine or other  
changes make this necessary). 

Build the controller plugin from its installation directory using CMake:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Point Gazebo to the plugin directory:
    $ cd build
    $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`

Then start the walker by typing (from the build directory):

    $ gazebo ../walker.world


