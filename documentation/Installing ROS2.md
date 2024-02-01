# Installing ROS2

`apt install ros2`, almost.

## Step 0: Install ROS-Humble
We use ROS Humble, since it is the current LTS version.
This requires Ubuntu 22.04 at least.

[The official install guide is always the best start](https://docs.ros.org/en/humble/Installation.html)

`apt install ros-humble-desktop-full` should be good for most.

Like with ROS1, setup a workspace. Usually `colcon_ws`.
Notice that ROS2 moved from catkin to colcon for its build process, and there are some minor differences from a user standpoint.



### Step 0.5: Tame colcon
Colcon, unlike catkin, cares about your current directory when building, by default.
This is a dumb default.
Put in `~/.colcon/defaults.yaml` the following, with paths changed to fit your setup:
```
{
    "build": {
        "symlink-install": true,
        "build-base": "/code/ros2_ws/build",
        "install-base": "/code/ros2_ws/install",
        "base-paths": ["/code/ros2_ws/src"]
    },
    "test": {
        "build-base": "/code/ros2_ws/build",
        "install-base": "/code/ros2_ws/install",
        "event-handlers": ["console_direct+"],
        "base-paths": ["/code/ros2_ws/src"],
    }
}
```
Now you can run `colcon build` and `colcon test` from anywhere in your system and it will create its files in the same place, all the time.
You might need to change this if you ever need multiple workspaces and don't want to delete the created folders.

You can also find this file [here](../docker/defaults.yaml)


## [Continue with porting](./Porting%20a%20package.md)

## [Continue with a new package](./Making%20a%20new%20package.md)