ar_tracker
==========

Multi Marker Tracking &amp; Positioning System - 6 DOF Integrated with ROS

install
=======

You need to add this line to your .bashrc (depends on the camera), can be generated using a script as explained below:
export ARTOOLKIT_CONFIG="v4l2src device=/dev/video0 use-fixed-fps=false ! ffmpegcolorspace ! capsfilter caps=video/x-raw-rgb,bpp=24 ! identity name=artoolkit ! fakesink"

You need to generate the camera_para.dat file, using the artoolkit installation_scripts that applies the patch, located in:
scripts folder
https://github.com/enekochan/installation-scripts
and then replace the one found into the data folder of the root directory

running
=======

rosrun ar_tracker ar_tracker (from inside the root directory of the package)
