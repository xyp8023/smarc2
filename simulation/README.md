# Simulation
This directory contains the SMaRC simulation environment submodules.
These might not be ROS packages.
**These should be [submodules](../documentation/Working%20with%20submodules.md)**



## SMARCUnity
The three submodules that start with SMARCUnity are meant to be opened or imported as Unity packages.
Depending on your goals, either the HDRP or the Standard project, with the Assets module on top is used.
Check their individual READMEs for details.

### HDRP
Uses the High Def. Render Pipeline to produce some good looking water and realistic waves. 
Runs fast enough for realtime usage while looking pretty.

### Standard
Uses the "Unity Standard" trading graphical fidelity for _speed_.
If you need to do some reinforcement learning or similar "try a million times" approaches, this is the way to go.

### Assets
Common package that contains all the sensors, prefabs, vehicles, etc.
Should be imported from the package manager in Unity.