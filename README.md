# Free Gait

**An Architecture for the Versatile Control of Legged Robots**

[![Free Gait Maneuvers with ANYmal](https://img.youtube.com/vi/EI1zBTYpXW0/0.jpg)](https://www.youtube.com/watch?v=EI1zBTYpXW0)

*Free Gait* is a software framework for the versatile, robust, and task-oriented control of legged robots. The Free Gait interface defines a *whole-body abstraction layer* to accomodate a variety of task-space control commands such as end effector, joint, and base motions. The deﬁned motion tasks are tracked with a feedback whole-body controller to ensure accurate and robust motion execution even under slip and external disturbances. The application of this framework includes intuitive tele-operation of the robot, efficient scripting of behaviors, and fully autonomous operation with motion and footstep planners.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Maintainer: Péter Fankhauser, pfankhauser@ethz.ch<br />
With contributions by: Samuel Bachmann, Philipp Krüsi, Dario Bellicoso, Christian Gehring, Remo Diethelm, Thomas Bi<br />
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

## FAQ

### Actions Are Not Found

If no Free Gait actions are found/loaded, this service call will return empty:

	rosservice call /free_gait_action_loader/list_actions "collection_id: ''"

In this case, try [initializing rosdep](http://wiki.ros.org/rosdep#Initializing_rosdep) with:

	sudo rosdep init
	rosdep update
