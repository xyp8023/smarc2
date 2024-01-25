# Submodules
We use some git submodules in this repo.
Here you can read why that is the case and how you can use them.

## Why?
Because some packages are optional under different setups.


Examples:
- You are developing a control algorithm, on Ubuntu 22.04 on your desktop, for long-distance missions with algae.
  - You want to test this algorihm.
  - To do so, you need to run it on simulation.
    - So you pull the simulation submodules.
    - These submodules can be kind of large and require a graphical interface.
      - Meaning they are useless on the real robot, and thus should not be pulled in there.
---

Or the other way around:

- You are developing a driver for a sonar system.
  - This is a driver, meaning it only makes sense when the hardware is present.
  - The hardware can not be connected to a desktop computer.
    - Thus this driver package should not be pulled on one.
    - It can be pulled on the real robot that has the hardware.

---
## How?
Assuming you have already cloned this repository.

### Add a new submodule to this repo
```bash
cd colcon_ws/smarc2
git submodule add https://github.com/link_to_repo path/to/local/repo
```

Example:
```bash
git submodule add https://github.com/KKalem/ROS-TCP-Endpoint external/ROS-TCP-Endpoint
```


