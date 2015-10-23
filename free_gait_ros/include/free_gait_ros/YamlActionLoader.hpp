/*
 * YamlActionLoader.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Georg Wiedebach
 */

#ifndef WHOLE_BODY_CLIMBING_SRC_YAMLACTIONLOADER_HPP_
#define WHOLE_BODY_CLIMBING_SRC_YAMLACTIONLOADER_HPP_

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <free_gait_ros/ActionBase.hpp>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace free_gate {

class YamlActionLoader {
public:
	YamlActionLoader(boost::filesystem::path file_path, std::string source_frame_id = "");
	virtual ~YamlActionLoader() {};

	free_gait_msgs::StepGoal getGoal();
	free_gait::ActionBase getAction(actionlib::SimpleActionClient<free_gait_msgs::StepAction>* client);

private:
	void parseFile();
	trajectory_msgs::MultiDOFJointTrajectory parseMultiDOFTrajectory(std::string joint_name, YAML::Node trajectory_parameters);
	trajectory_msgs::JointTrajectory parseJointTrajectory(YAML::Node trajectory_parameters);
	void adaptCoordinates(free_gait_msgs::StepGoal& goal, tf::Transform transform);

	boost::filesystem::path file_path_;
	std::string source_frame_id_;
	free_gait_msgs::StepGoal goal_;
};

} /* namespace free_gate */

#endif /* WHOLE_BODY_CLIMBING_SRC_YAMLACTIONLOADER_HPP_ */
