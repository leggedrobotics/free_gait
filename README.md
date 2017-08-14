# Free Gait

**An Architecture for the Versatile Control of Legged Robots**

[![Free Gait Applications](http://i.imgur.com/fsxbJNn.gif)](https://www.youtube.com/watch?v=EI1zBTYpXW0)

*Free Gait* is a software framework for the versatile, robust, and task-oriented control of legged robots. The Free Gait interface defines a *whole-body abstraction layer* to accommodate a variety of task-space control commands such as end effector, joint, and base motions. The deﬁned motion tasks are tracked with a feedback whole-body controller to ensure accurate and robust motion execution even under slip and external disturbances. The application of this framework includes intuitive tele-operation of the robot, efficient scripting of behaviors, and fully autonomous operation with motion and footstep planners.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Maintainer: Péter Fankhauser, pfankhauser@ethz.ch<br />
With contributions by: Samuel Bachmann, Dario Bellicoso, Thomas Bi, Remo Diethelm, Christian Gehring<br />
Affiliation: Robotic Systems Lab, ETH Zurich**

## Publications

If you use this work in an academic context, please cite the following publication(s):

* P. Fankhauser, D. Bellicoso, C. Gehring, R. Dubé, A. Gawel, M. Hutter,
**"Free Gait – An Architecture for the Versatile Control of Legged Robots"**,
in IEEE-RAS International Conference on Humanoid Robots, 2016. ([PDF](https://www.researchgate.net/publication/312111333))

        @inproceedings{Fankhauser2016FreeGait,
            author = {Fankhauser, P{\'{e}}ter and Bellicoso, C. Dario and Gehring, Christian and Dub{\'{e}}, Renaud and Gawel, Abel and Hutter, Marco},
            booktitle = {IEEE-RAS International Conference on Humanoid Robots},
            title = {{Free Gait – An Architecture for the Versatile Control of Legged Robots}},
            year = {2016},
        }

## Unit Tests

	catkin build free_gait_core --no-deps --verbose --catkin-make-args run_tests

## Overview

[This video](https://www.youtube.com/watch?v=EI1zBTYpXW0) shows some of the applications of Free Gait.

| [![Free Gait Notions and Coordinate Systems](free_gait_core/doc/notions_and_coordinate_systems_preview.png)](free_gait_core/doc/notions_and_coordinate_systems.pdf) | [![Free Gait Motion Examples](free_gait_core/doc/motion_examples_preview.jpg)](free_gait_core/doc/motion_examples.jpg) | [![Free Gait Control Scheme](free_gait_core/doc/control_scheme_preview.png)](free_gait_core/doc/control_scheme.pdf) |
| --- | --- | --- |
| Motions are based on a combination of (possibly multiple) leg motions and a base motion per command (step). | The command structure allows to control legged robots in various ways. | Motion  goals  are  commanded  through  the  Free  Gait  API  to  the whole-body  motion  controller. |

## Usage

[This video](https://www.youtube.com/watch?v=PNJZvCCOmD8) presents an overview over the tools provided in Free Gait.

### Free Gait Actions

Free Gait actions are libraries and scripts that define motions using the Free Gait API (for ROS ). For ROS, these actions can be write in any language with a ROS client library using the message and action definitions in *[free_gait_msgs]*. For C++, Free Gait provides the *[free_gait_core]* library to work with motion definitions and the *[free_gait_ros]* for interfacing ROS. To work with Python, use the *[free_gait_python]* library. For simple motion definitions, Free Gait supports actions defined in [YAML] format. For more information about using YAML actions, refer to [YAML Scripting Interface](#yaml-scripting-interface).

Free Gait actions can be either launched manually or with help of the *[free_gait_action_loader]*.

### Free Gait Action Loader

The *[free_gait_action_loader]* allows to launch actions through a [ROS service](free_gait_msgs/srv/SendAction.srv) or [ROS action](free_gait_msgs/action/ExecuteAction.action). Currently, the action loader supports *YAML* motion definitions, *Python* scripts, and starting *ROS launch files* for *C++* and other libraries.

Run the *[free_gait_action_loader]* with
	
	rosrun free_gait_action_loader action_loader.py

The *[free_gait_action_loader]* manages the actions and makes sure that only one action is running at the time. To register an action with the *[free_gait_action_loader]*, one has to load the action as a ROS plugin.

### RQT Free Gait Action

<a href="https://www.youtube.com/watch?v=PNJZvCCOmD8"><img align="right" src="http://i.imgur.com/sy91C8f.gif"></a>

The *[rqt_free_gait_action]* package provides a [rqt](http://wiki.ros.org/rqt) user interface to the  *[free_gait_action_loader]*. The interface shows the actions organized in *collections* and allows to preview and send the actions. Additionally, collections can be run as sequence of actions and with a right-click, a collection can be selected as favorite. Favorites are shown as buttons on the top for quick access.

<br/>
<br/>
<br/>

### RQT Free Gait Monitor

<a href="https://www.youtube.com/watch?v=PNJZvCCOmD8"><img align="right" src="http://i.imgur.com/vIW2fjj.gif"></a>

Once actions are being executed by the *[Free Gait action server](free_gait_ros/include/free_gait_ros/FreeGaitActionServer.hpp)*, the *[rqt_free_gait_monitor]* shows the progress of the execution and allows to pause and stop the active action.

<br/>
<br/>
<br/>
<br/>
<br/>

### Free Gait RViz Plugin

<a href="https://www.youtube.com/watch?v=PNJZvCCOmD8"><img align="right" src="http://i.imgur.com/GUqRKD4.gif"></a>

Actions can be previewed with the *[free_gait_rviz_plugin]*. It takes the current state of the robot and visualizes the motion based on the defined action. This [RViz] plugin allows to scrub through time and visualize footholds, trajectories, support polygons etc.

<br/>
<br/>
<br/>
<br/>

### YAML Scripting Interface

<a href="https://www.youtube.com/watch?v=PNJZvCCOmD8"><img align="right" src="http://i.imgur.com/N9L6ogg.gif"></a>

For simple motion sequences, Free Gait actions can be defined as  a sequence of [YAML] definitions.

For example, this action lifts the right-fore leg of the robot:

    adapt_coordinates:
     - transform:
        source_frame: footprint
        target_frame: odom

    steps:
     - step:
       - base_auto:
     - step:
       - base_auto:
       - end_effector_target:
          name: RF_LEG
          ignore_contact: true
          target_position:
           frame: footprint
           position: [0.39, -0.22, 0.20]

The `adapt_coordinates` command transforms motions defined in the `source_frame` to the `target_frame`.

## FAQ

### Actions Are Not Found

If no Free Gait actions are found/loaded, this service call will return empty:

	rosservice call /free_gait_action_loader/list_actions "collection_id: ''"

In this case, try [initializing rosdep](http://wiki.ros.org/rosdep#Initializing_rosdep) with:

	sudo rosdep init
	rosdep update


[ROS]: http://www.ros.org
[RViz]: http://wiki.ros.org/rviz
[YAML]: http://yaml.org
[free_gait_core]: free_gait_core
[free_gait_ros]: free_gait_ros
[free_gait_python]: free_gait_python
[free_gait_msgs]: free_gait_msgs
[free_gait_action_loader]: free_gait_action_loader
[rqt_free_gait_action]: rqt_free_gait_action
[rqt_free_gait_monitor]: rqt_free_gait_monitor
[free_gait_rviz_plugin]: free_gait_rviz_plugin
