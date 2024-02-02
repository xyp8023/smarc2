#!/usr/bin/python3
import os, subprocess, sys, time, signal

# Make sure we run in /smarc2
in_smarc2 = True
if(os.getcwd()[-6:] != "smarc2"):
    in_smarc2 = False
    print("Are you running this in scripts...?")
    if(os.getcwd()[-14:] == "smarc2/scripts"):
        print("Yes you are...")
        os.chdir("..")
        in_smarc2 = True

if(not in_smarc2): sys.exit(1)



# We want to find all launch files that matter
# Colcon already knows this, so we ask it...
res = subprocess.run(["colcon", "list"], stdout=subprocess.PIPE)
packages_lines = res.stdout.decode().split("\n")
packages = []
for p in packages_lines:
    parts = p.split("\t")
    if(len(parts) > 1): packages.append((parts[0], parts[1]))


# packages = [(package_name, package_path), ...]
# now find all the launch files in this package
packages_launches = []
for p_name, p_path in packages:
    res = subprocess.run(["find", p_path, "-name", "*.launch"], stdout=subprocess.PIPE)
    launches_lines = res.stdout.decode().split("\n")
    launchfiles = [l for l in launches_lines if l != '']
    if(len(launchfiles) != 0):
        packages_launches.append((p_name, launchfiles))

# packages_launches = [(package_name, [..launch, ..launch]), ...]
# to launch these one by one, and then run some ros commands for each package
for p_name, launchfiles in packages_launches[:1]:
    for launchfile in launchfiles:
        # we want these in the BG and killable
        launched = subprocess.Popen(["ros2", "launch", launchfile], stdout=subprocess.DEVNULL)
        # wait a little for all the stuff to run...
        wait_seconds = 10
        print(f"Waiting for launch to launch for {wait_seconds}s")
        for i in range(wait_seconds):
            print(f"{i+1}/{wait_seconds}")
            time.sleep(1)

        # then we run ros2 node list to get all the nodes this launch file runs
        res = subprocess.run(["ros2", "node", "list"], stdout=subprocess.PIPE)
        node_list = res.stdout.decode().split("\n")
        node_list = [n for n in node_list if n != ""]
        print(f"Package '{p_name}' has '{launchfile}' which launches:")
        for node in node_list: print(node)

        # and then we kill that launch process by force
        # dont want random nodes chilling around
        # between different launches...
        # TODO make sure these are killed proper...?
        print(f"Killing {launchfile}")
        launched.send_signal(signal.SIGINT)
        launched.wait(timeout=30)
        launched.kill()
        


