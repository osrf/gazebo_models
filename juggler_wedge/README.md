# This model implements the ball-in-wedge juggler described in:

Stefan Schaal and Chris G. Atkeson. Open loop stable control strategies for robot juggling. In Proc. IEEE Intl. Conf. Robot. Autom. (ICRA), 1993.

# Building the controller for the juggler:

Build the controller for the juggler from its installation directory using CMake:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Point Gazebo to the plugin directory:
    $ cd build
    $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`

Then start the juggling process by typing (from the build directory):

    $ gazebo ../juggler_wedge.world

# Searching for parameters:

For new physics engines- or getting existing physics working with
the wedge juggler- it may take a search to find reasonable
parameters.

Replace the text ''libjuggler.so'' with ''libjuggler-search.so'' in **model.sdf** 
and run the juggler as usual. For demonstration purposes, we will tell Gazebo to use an alternative physics engine. For example:

    $ gazebo -e dart ../juggler.world 

will use the DART physics engine and attempt to find good parameter settings
for the juggler. The method used is grid search, and the search will likely take
several hours (**for maximum speed, we recommend setting Gazebo's real time
physics update to ''0'' in either the GUI or the **model.sdf** file**). Before running the grid search you will need to set
two environment variables:

    $ export GZ_BRUTE_PARAM_FILE=params
    $ export GZ_BRUTE_OUTPUT_FILE=output

The first line sets the search ranges and steps. The second line indicates 
where the evaluation results should be placed. The grid search will terminate 
Gazebo when complete. Assuming that your file is named 'output', you will 
likely want to sort it to find parameters that yield juggling, like so:

    $ sort -g -k6 -k5 < output

The tail of this output should look something like this:

    bounce:0.10000000000000001:ball-geom mu:0:ball-geom limit:-0.27488999999999997 period:12 20000 1.71585

This means that the parameters bounce=0.1, mu=0.0, limit=-0.275, and 
period=12.0 yielded a juggle for 20000 iterations (the maximum used in the
search) with a mean ball height of 1.71585. If the juggle does not last 20000 
iterations, the ball was ejected from the juggler some time before. If the
mean height for the juggle is lower- say 1.2- then the ball became trapped
at the bottom of the wedge (there was no or little ballistic flight phase
after some time).


# Setting the found parameters in the juggler
The bounce (''bounce'') and friction (''mu'') parameters should be set in the appropriate (i.e., possibly physics-engine dependent) part of the **model.sdf** 
file. The limit and period should be set in **controller.cpp** (lines 32 and
33, respectively). After **controller.cpp** is updated:

1. Re-build the plugin (''make'')
2. Replace the text ''libjuggler-search.so'' back with ''libjuggler.so'' in **model.sdf**
3. Run the juggler as before ('gazebo -e dart ../juggler.world')

