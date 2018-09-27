#include <iostream>
#include <free_gait_msgs/Step.h>
#include <ros/node_handle.h>
#include <string>
#include <free_gait_core/base_motion/BaseAuto.hpp>
#include <free_gait_ros/free_gait_ros.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_sine");
    ros::NodeHandle n;

    free_gait::AdapterRos adapter(n);
    free_gait::StepRosConverter converter(adapter.getAdapter());

    ros::Rate r(10);

    int a = 0;
    while(n.ok()){
        free_gait::FreeGaitActionClient client(n);
        client.registerCallback();
        
        free_gait_msgs::ExecuteStepsGoal goal;
        free_gait::Step step;
        free_gait::BaseAuto base_auto;
        free_gait_msgs::Step step_msg;

        double height = 0.35 + std::sin(2*M_PI * (double)a++ * 0.01) * 0.10;
        std::cout << "height: " <<  height << std::endl;

        base_auto.setHeight(height);
        // add the base auto motion to the step
        step.addBaseMotion(base_auto);
        // convert the step to ROS message format
        converter.toMessage(step, step_msg);
        
        // add the ROS message to the queue of steps for the goal
        goal.steps.push_back(step_msg);
        
        // send the goal to the free gait server
        client.sendGoal(goal, false);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

