# Scripts
A pile of bash or python scripts to make life easier.

List them here with a simple explanation so people know what they are running ;)

### get-submodules.sh
Usage:
```bash
cd colcon_ws/smarc2
./scripts/get-submodules.sh <foldername>
```
where `foldername` is a folder (or its first few characters, like `ext` for `external`).

Example: `./scipts/get-submodules.sh ext` will update all submodules in the folder `smarc2/external`. `./scripts/get-submodules.sh sim` will do the same for `smarc2/simulation`.

### rosdep_install_from_src.sh
A single line to install all the dependencies in the `src` directory that aren't sam- lolo- or smarc- named.

Usage:
```bash
cd colcon_ws
./smarc2/scripts/rosdep_install_from_src.sh
```

### unity_ros_bridge.sh
Runs the Unity ROS-TCP-Endpoint with default args for local use. Run from where-ever.