# zoidberg_nav
navigation for zoidberg

Install
-------

This package is installed using catkin_make. Clone this repository into the

    catkin_workspace/src/

directory (fill in catkin_workspace with your specific catkin workspace
directory). The basic instructions for making the workspace are found in the
[Ros tutorial][RosT1].

Once the zoiderg_nav package is in place, move to the base catkin_workspace
and run catkin_make

```
cd catkin_workspace
catkin_make
```

Once installed, this package defines a number of ROS messages related to
 navigation and a few ROS nodes. A launch file is used to set up navigation.

```
roslaunch zoidberg_nav.launch
```

This sets up a node "helm" that sits around and waits for commands

Currently the script
```
rosrun zoidberg_nav navigation_client.py
```

is used to send commands. Since there is no feedback from the sensors it currently just times out
on all commands, but hopefully that will change soon.

(for the example of the navigation server). Possible scripts avalible to this
command can be accessed by tab completion, i.e
```
rosrun zoidberg_nav *tab* *tab*
```


[RosT1]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


