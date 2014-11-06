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


Marker info and visualization only
----------------------------------

Run the command below in your computer (you can set inside the launch file to turn on visualization or not, visualization can be run in a different computer now): 

`roslaunch phase_space phase_space.launch`


Tracking bodies with known-positioned leds
------------------------------------------

You need to:
- Write an OBJECT.YAML configuration file in the `config` folder with the leds id and local coordinates of them. A CAD tool is advised if the object model is available, and DOUBLE CHECK! the correspondence id-coordinates using the master tool in the phasepace).
- Provide an OBJECT.STL model (only for online visualization), and place it in the `urdf/mesh` folder
- Edit `track_object.launch`, specifically the line `<arg name="object" default="OBJECT" />`

IMPORTANT: note that OBJECT is repeated in the three cases.

Finally, run:

`roslaunch phase_space track_object.launch`

ToDo: write procedure to track multiple objects


Tracking bodies using the star
------------------------------

This requires a calibration procedure in order to known the transformation between the star and the object. The example will be the cup.

ToDo: the calibration procedure, it will require object pose estimation, and kinect-phase_space calibration

