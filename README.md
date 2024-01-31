# SMaRC 2 for ROS 2 Humble
This is the repository of packages for ROS Humble.
Each of the following directories also include more detailed readmes within.

Most of the dependendencies can be installed with rosdep: `rosdep install -i --from-path src --rosdistro humble -y --skip-keys="sam_* lolo_* smarc_*`.


## Behaviours
High-level constructs like action servers and behaviour trees.
These usually rely on many other packages to function.

## Docker
Docker-related stuff. Like dockerfiles and other such artifacts.

## Documentation
A good starting point if you would like to start writing some code to run on a robot.
Check [this basic tutorial on creating a package](./documentation/Making%20a%20new%20package.md) to start your journey into Ros Humble.

Images, diagrams, pdfs, text that describe larger concepts that apply to multiple packages.
Tutorials and such also go here.

Individual packages should have their own documentation in their readme files.

## Examples
Where example nodes and code go. These are not meant to be ran on a robot, and usually their performance is not the point.
If you had to give a piece of code out more than once, it should live here.

## External
Packages that we have not developed ourlseves entirely, but rely on.
Maybe with edits from us that are not available upstream.
**Always check the license of the package you put here!**

## Gui
Packages related to external-to-robot control and monitoring. 
Stuff like MQTT and node-red configurations.

## Messages
All ROS message, action and service definitions should live here.
Put them in appropriate packages, like sam-hardware specific stuff in sam_msgs and so on.

Splitting messages into multiple packages depending on their use context is very, very welcome.
If some messages are basically used only "internally", that set of messages should be in their own package here.
An example would be "BT messages" that only the behaviour tree interacts with. Or "SLAM messages" used for communication between some slam-related nodes and nowhere else.

## Robot descriptions
URDF files and the accompanying models of individual robots.
These are depended on by many packages and should require no dependencies.


## Sam
SAM's hardware-specific packages. 
It is likely that these packages will not compile or run on anything but the real SAM hardware.
**These should be set up as submodules.**

## Scripts
Where scripts for ease-of-use are stored.

## Simulation
This is where our simulation-related packages live.
If something is required ONLY for the sim and not anywhere else, that thing should be in here.
Similar to Sam, **these are submodules.**
From the perspective of ROS, simulation is just another robot.




