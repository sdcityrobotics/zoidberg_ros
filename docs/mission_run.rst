Run catkin_make:

```
cd catkin_workspace
catkin_make
```

read the zoidberg_ros.launch file. This shows you proportionality constants and
the pixhawk channels being used for mission stuff. If a channel is being used,
it is remapped from /apm to its own channel. For instance,

```
<remap to="/depth" from="/apm/global_position/rel_alt"/>
```

maps the channel /apm/global_position/rel_alt to the channel /depth. This makes
it simpler to type, but more importantly identifies the depth channel that we
are using for all navigation purposes. Please check that all the channel mapppings
are correct, i.e. the depth sensor we are actually using is publishing to this
channel.

Launch zoidberg_ros server

```
roslaunch zoidberg_ros zoidberg_ros.launch
```

This populates a lot of rostopics and rosservices. These can be seen with the command

```
rostopic list
```

A single topic stream can be view with, i. e.

```
rostopic echo /rcout
```

The rcout channel is particulary important, this indicates that active motor commands
are in fact being sent from the Pixhawk motor controller.

Use rosrun to run a mission. The file basic_mission is a good template

```
rosrun zoidberg_ros basic_mission.py
```

This file can be found in the zoidberg_ros/scripts folder. It shows all of the
avalible behaviors implimented on the robot, and these can be mixed and matched.
