# Docker
We use docker containers to run tests and such. 

> Maybe we can use containers for all the nodes (or groups of them) as well?
> ROS2 is p2p, so we could have "core" in a container, "slam" in one etc. 

## Tame docker
Using sudo all the time is annoying, follow [this guide](https://docs.docker.com/engine/install/linux-postinstall/)

## Cheatsheet
**Build image from dockerfile:** `docker build - -t <name>:<tag> < <path/to/dockerfile>`

Add `--no-cache` to make it build from scratch.

**Check your images:** `docker images`

An image is a _description_ of what will exist in a container when it is run. Basically a "class", doesn't do anything until its instantiated with its constructor.

**Container from image:** `docker run <name>/<tag>`

**Check your containers:** `docker ps`

**Nuke a container:** `docker remove <container_name>`

A container is an instance of an image. It does things, has files and such. There can be many containers from the same image that do not share anything except their initial state.

* To give container a name: add `--name <name>` to `run`
* To share a folder between host and container: `--mount type=bind,source=<folder path on host>,target=<folder path on container>` (you can use `"$pwd"` in the host path part)
* To make container interactive (so that it doesn't just run and quit): add `-it` flag to `run`, or `exec`

**Start a container that was run before:** `docker start <container_name>`

**Attach to a container that is alredy running:** `docker attach <container_name>`

Examples:
- Create a new container and run bash in it interactively: `docker run -it <container_name> /bin/bash`
- Run bash in a container that is already started: `docker exec -it <container_name> /bin/bash`

### Common problems
- "When I do `docker start`, it quits right away."
  - You did not give the container a command to run when you did `docker run`. This is usually the case when the entrypoint of the image is not a long-running (or interactive) program. If you used the `dockerfile` in this repo, then you should run `bash` at least. See example above.


### End to end example
You can find an example of using docker with the sim with all the commands [here](../simulation/README.md).
