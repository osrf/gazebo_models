# This model implements the modification to the Shannon juggler described in:

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

    $ gazebo ../juggler_schaal.world

# Alternative physics engines:

This example requires restitution to run successfully. Only ODE's restitution
model in Gazebo has been tuned to work with this example (circa Gazebo 4.0).
Bullet's restitution model should work with few changes. It remains 
to make restitution parameters settable through API calls for Gazebo's other 
supported physics engines (DART and Simbody at the time of this writing).

# Searching for parameters:

For new physics engines- or getting existing physics working with
the Schaal juggler- it may take a parameter search to find reasonable
parameters (this is assuming that the API supports setting collision 
restitution, i.e., partially or fully elastic impacts).

and run the juggler as usual. For demonstration purposes, we will tell Gazebo to use an alternative physics engine. For example:

    $ gazebo -e dart ../juggler_schaal.world 

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

    $ sort -g -k5 < output

The tail of this output should look something like this:

    bounce:1:ball-geom mu:0:ball-geom limit:0.59000000000000008 period:2.2000000000000002 20000

This means that the parameters bounce=1.0, mu=0.0, limit=0.59, and 
period=2.2 yielded a juggle for 20000 iterations (the maximum used in the
search). If the juggle does not last 20000 iterations, the juggling failed 
some time before. 

# Setting the found parameters in the juggler
The parameters should be set in the **controller.cpp** (lines 55-58). After 
**controller.cpp** is updated:

1. Re-build the plugin (''make'')
2. Replace the text ''libjuggler-search.so'' back with ''libjuggler.so'' in **model.sdf**
3. Run the juggler as before ('gazebo -e dart ../juggler_schaal.world')
 
