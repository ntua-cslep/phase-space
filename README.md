Phase-space ROS package
=======================

Software related to the use of the Phase Space motion tracking.

1. Trun on the phase-space system: 

`username: demo`

`password:demo`

`: startx`

2. Calibrate the phase space system

Run the command below in the phase space system:
`cd phasespace`
`./calib`

it opens a window which you can use it for calibrating system.
you should use the calibration wand!  (http://www.phasespace.com/)
define the origin and axises. 


Using marker info only
----------------------

Run the command below in your computer (you can set inside the launch file to turn on visualization or not, visualization can be run in a different computer now): 

`roslaunch phase_space phase_space.launch`


Tracking bodies
---------------

Requisites: 
- The leds id and location in the reference system of the object (see inside `config` for examples, a CAD tool is advised if the object model is available, and DOUBLE CHECK! the correspondence id-coordinates)
- An STL model of the object (only for online visualization) into the urdf/mesh folder
- The macro and urdf file of your object: urdf/macro/{object_name}.urdf (the macro definition) and urdf/{object_name}.urdf.xacro (the world that instantiate the macro)

This node will publish the object pose w.r.t. the phase space world.




