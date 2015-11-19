/*
 * YamlActionLoader.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Georg Wiedebach
 */

#include "free_gait_ros/YamlActionLoader.hpp"

namespace free_gate {

YamlActionLoader::YamlActionLoader(boost::filesystem::path file_path, std::string source_frame_id) {
	file_path_ = file_path;
	source_frame_id_ = source_frame_id;
}

free_gait_msgs::ExecuteStepsGoal YamlActionLoader::getGoal() {
	try {
		parseFile();
	} catch(YAML::ParserException& e) {
		ROS_INFO("Failed to parse file: %s", file_path_.c_str());
		ROS_INFO("%s", e.what());
	}

	return goal_;
}

free_gait::ActionBase YamlActionLoader::getAction(actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>* client) {
	free_gait::ActionBase action(client, getGoal());
	return action;
}

void YamlActionLoader::parseFile() {
	YAML::Node file = YAML::LoadFile(file_path_.c_str());
	free_gait_msgs::ExecuteStepsGoal goal;

	for (YAML::Node step_parameter : file["steps"]) {
		if (!step_parameter["step"]) {
			continue;
		}

		step_parameter = step_parameter["step"];
		free_gait_msgs::Step step;

		for (YAML::Node motion_parameter : step_parameter) {
			if (motion_parameter["footstep"]) {
				step.footstep.push_back(parseFootstep(motion_parameter["footstep"]));
			}
			if (motion_parameter["joint_trajectory"]) {
				step.joint_trajectory.push_back(parseJointTrajectory(motion_parameter["joint_trajectory"]));
			}
			if (motion_parameter["base_auto"]) {
				step.base_auto.push_back(parseBaseAuto(motion_parameter["base_auto"]));
			}
		}

		goal.steps.push_back(step);
	}

//	int step_number = 0;
//	for (YAML::Node steps : file["steps"]) {
//		step_number++;
//		YAML::Node step_parameter = steps["step"];
//		free_gait_msgs::Step step;
//		step.step_number = step_number;
//
//		if (step_parameter["swing_data"]) {
//			for (YAML::Node swing_data_parameter : step_parameter["swing_data"]) {
//				free_gait_msgs::SwingData swing_data;
//				swing_data.name = swing_data_parameter["name"].as<std::string>();
//
//				if (swing_data_parameter["type"]) {
//					swing_data.type = swing_data_parameter["type"].as<std::string>();
//				}
//
//				if (swing_data_parameter["profile"]) {
//					swing_data.profile.target.point.x = swing_data_parameter["profile"]["target"][0].as<double>();
//					swing_data.profile.target.point.y = swing_data_parameter["profile"]["target"][1].as<double>();
//					swing_data.profile.target.point.z = swing_data_parameter["profile"]["target"][2].as<double>();
//
//					if (swing_data_parameter["profile"]["target_frame"]) {
//						swing_data.profile.target.header.frame_id = swing_data_parameter["profile"]["target_frame"].as<std::string>();
//					}
//
//					if (swing_data_parameter["profile"]["height"]) {
//						swing_data.profile.height = swing_data_parameter["profile"]["height"].as<double>();
//					}
//
//					if (swing_data_parameter["profile"]["duration"]) {
//						ros::Duration duration(swing_data_parameter["profile"]["duration"].as<double>());
//						swing_data.profile.duration = duration;
//					}
//
//					if (swing_data_parameter["profile"]["type"]) {
//						swing_data.profile.type = swing_data_parameter["profile"]["type"].as<std::string>();
//					}
//				}
//
//				if (swing_data_parameter["foot_trajectory"]) {
//					swing_data.foot_trajectory = parseMultiDOFTrajectory(swing_data.name, swing_data_parameter["foot_trajectory"]);
//				}
//
//				if (swing_data_parameter["joint_trajectory"]) {
//					swing_data.joint_trajectory = parseJointTrajectory(swing_data_parameter["joint_trajectory"]);
//				}
//
//				if (swing_data_parameter["no_touchdown"]) {
//					swing_data.no_touchdown = swing_data_parameter["no_touchdown"].as<bool>();
//				}
//
//				if (swing_data_parameter["surface_normal"]) {
//					swing_data.surface_normal.vector.x = swing_data_parameter["surface_normal"][0].as<double>();
//					swing_data.surface_normal.vector.y = swing_data_parameter["surface_normal"][1].as<double>();
//					swing_data.surface_normal.vector.z = swing_data_parameter["surface_normal"][2].as<double>();
//				}
//
//				if (swing_data_parameter["ignore_for_pose_adaptation"]) {
//					swing_data.ignore_for_pose_adaptation = swing_data_parameter["ignore_for_pose_adaptation"].as<bool>();
//				}
//
//				step.swing_data.push_back(swing_data);
//			}
//		}
//
//		if (step_parameter["base_shift_data"]) {
//			for (YAML::Node base_shift_data_parameter : step_parameter["base_shift_data"]) {
//				free_gait_msgs::BaseShiftData base_shift_data;
//				base_shift_data.name = base_shift_data_parameter["name"].as<std::string>();
//
//				if (base_shift_data_parameter["ignore"]) {
//					base_shift_data.ignore = base_shift_data_parameter["ignore"].as<bool>();
//				}
//
//				if (base_shift_data_parameter["type"]) {
//					base_shift_data.type = base_shift_data_parameter["type"].as<std::string>();
//				}
//
//				if (base_shift_data_parameter["profile"]) {
//					if (base_shift_data_parameter["profile"]["target"])
//					{
//						YAML::Node target = base_shift_data_parameter["profile"]["target"];
//						if (target["position"]) {
//							base_shift_data.profile.target.pose.position.x = target["position"][0].as<double>();
//							base_shift_data.profile.target.pose.position.y = target["position"][1].as<double>();
//							base_shift_data.profile.target.pose.position.z = target["position"][2].as<double>();
//						}
//
//						if (target["orientation"]) {
//							if (target["orientation"].size() == 4) {
//								base_shift_data.profile.target.pose.orientation.x = target["orientation"][0].as<double>();
//								base_shift_data.profile.target.pose.orientation.y = target["orientation"][1].as<double>();
//								base_shift_data.profile.target.pose.orientation.z = target["orientation"][2].as<double>();
//								base_shift_data.profile.target.pose.orientation.w = target["orientation"][3].as<double>();
//							}
//							if (target["orientation"].size() == 3) {
//								tf::Quaternion quaternion;
//								double roll  = target["orientation"][0].as<double>();
//								double pitch = target["orientation"][1].as<double>();
//								double yaw   = target["orientation"][2].as<double>();
//								quaternion.setRPY(roll, pitch, yaw);
//								base_shift_data.profile.target.pose.orientation.x = quaternion.x();
//								base_shift_data.profile.target.pose.orientation.y = quaternion.y();
//								base_shift_data.profile.target.pose.orientation.z = quaternion.z();
//								base_shift_data.profile.target.pose.orientation.w = quaternion.w();
//							}
//						}
//					}
//
//
//					if (base_shift_data_parameter["profile"]["target_frame"]) {
//						base_shift_data.profile.target.header.frame_id = base_shift_data_parameter["profile"]["target_frame"].as<std::string>();
//					}
//
//					if (base_shift_data_parameter["profile"]["height"]) {
//						base_shift_data.profile.height = base_shift_data_parameter["profile"]["height"].as<double>();
//					}
//
//					if (base_shift_data_parameter["profile"]["duration"]) {
//						ros::Duration duration(base_shift_data_parameter["profile"]["duration"].as<double>());
//						base_shift_data.profile.duration = duration;
//					}
//
//					if (base_shift_data_parameter["profile"]["type"]) {
//						base_shift_data.profile.type = base_shift_data_parameter["profile"]["type"].as<std::string>();
//					}
//				}
//
//				if (base_shift_data_parameter["trajectory"]) {
//					base_shift_data.trajectory = parseMultiDOFTrajectory("base", base_shift_data_parameter["trajectory"]);
//				}
//
//				step.base_shift_data.push_back(base_shift_data);
//			}
//		}
//
//		if (step_parameter["ignore_base_shift"]) {
//			step.ignore_base_shift = step_parameter["ignore_base_shift"].as<bool>();
//		}
//
//		goal.steps.push_back(step);
//	}

	if (file["adapt_coordinates"]) {
		YAML::Node adapt_parameters = file["adapt_coordinates"];
		std::string target_frame_id = adapt_parameters["frame"].as<std::string>();
		tf::Vector3 position(0.0, 0.0, 0.0);
		tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);

		if (adapt_parameters["pose"]) {
			position[0] = adapt_parameters["pose"]["position"][0].as<double>();
			position[1] = adapt_parameters["pose"]["position"][1].as<double>();
			position[2] = adapt_parameters["pose"]["position"][2].as<double>();

			orientation[0] = adapt_parameters["pose"]["orientation"][0].as<double>();
			orientation[1] = adapt_parameters["pose"]["orientation"][1].as<double>();
			orientation[2] = adapt_parameters["pose"]["orientation"][2].as<double>();
			orientation[3] = adapt_parameters["pose"]["orientation"][3].as<double>();
		}

		tf::Transform original_transform, transform;
		original_transform.setOrigin(position);
		original_transform.setRotation(orientation);

		tf::TransformListener listener;
		ros::Duration(1.0).sleep();
		tf::StampedTransform frame_transform;
		try {
			listener.lookupTransform(source_frame_id_, target_frame_id, ros::Time(0), frame_transform);
		} catch (tf::TransformException& e) {
			ROS_WARN("Could not lookup transform from %s to %s.", source_frame_id_.c_str(), target_frame_id.c_str());
			return;
		}

		transform.mult(frame_transform, original_transform);
		adaptCoordinates(goal, transform);
	}

	ROS_INFO("Parsed YAML file: goal has %i steps.", goal.steps.size());
	goal_ = goal;
}

free_gait_msgs::Footstep YamlActionLoader::parseFootstep(YAML::Node yaml_object) {
	free_gait_msgs::Footstep footstep;
	if (!yaml_object) {
		return footstep;
	}
	if (yaml_object["name"]) {
		footstep.name = yaml_object["name"].as<std::string>();
	}
	if (yaml_object["target"]) {
		footstep.target = parsePoint(yaml_object["target"]);
	}
	if (yaml_object["profile_height"]) {
		footstep.profile_height = yaml_object["profile_height"].as<double>();
	}
	if (yaml_object["average_velocity"]) {
		footstep.average_velocity = yaml_object["average_velocity"].as<double>();
	}
	if (yaml_object["profile_type"]) {
		footstep.profile_type = yaml_object["profile_type"].as<std::string>();
	}
	if (yaml_object["ignore_contact"]) {
		footstep.ignore_contact = yaml_object["ignore_contact"].as<bool>();
	}
	if (yaml_object["surface_normal"]) {
		footstep.surface_normal = parseVector(yaml_object["surface_normal"]);
	}
	if (yaml_object["ignore_for_pose_adaptation"]) {
		footstep.ignore_for_pose_adaptation = yaml_object["ignore_for_pose_adaptation"].as<bool>();
	}
	return footstep;
}

free_gait_msgs::JointTrajectory YamlActionLoader::parseJointTrajectory(YAML::Node yaml_object) {
	free_gait_msgs::JointTrajectory joint_trajectory;
	if (!yaml_object) {
		return joint_trajectory;
	}
	if (yaml_object["name"]) {
		joint_trajectory.name = yaml_object["name"].as<std::string>();
	}
	if (yaml_object["trajectory"]) {
		joint_trajectory.trajectory = parseJointTrajectories(yaml_object["trajectory"]);
	}
	if (yaml_object["ignore_contact"]) {
		joint_trajectory.ignore_contact = yaml_object["ignore_contact"].as<bool>();
	}
	if (yaml_object["surface_normal"]) {
		joint_trajectory.surface_normal = parseVector(yaml_object["surface_normal"]);
	}
	return joint_trajectory;
}

free_gait_msgs::BaseAuto YamlActionLoader::parseBaseAuto(YAML::Node yaml_object) {
	free_gait_msgs::BaseAuto base_auto;
	if (!yaml_object) {
		return base_auto;
	}
	if (yaml_object["height"]) {
		base_auto.height = yaml_object["height"].as<double>();
	}
	if (yaml_object["average_linear_velocity"]) {
		base_auto.average_linear_velocity = yaml_object["average_linear_velocity"].as<double>();
	}
	if (yaml_object["average_angular_velocity"]) {
		base_auto.average_angular_velocity = yaml_object["average_angular_velocity"].as<double>();
	}
	if (yaml_object["support_margin"]) {
		base_auto.support_margin = yaml_object["support_margin"].as<double>();
	}
	return base_auto;
}

geometry_msgs::PointStamped YamlActionLoader::parsePoint(YAML::Node yaml_object) {
	geometry_msgs::PointStamped point;
	if (yaml_object["frame"]) {
		point.header.frame_id = yaml_object["frame"].as<std::string>();
	}
	if (yaml_object["position"]) {
		point.point.x = yaml_object["position"][0].as<double>();
		point.point.y = yaml_object["position"][1].as<double>();
		point.point.z = yaml_object["position"][2].as<double>();
	}
	return point;
}

geometry_msgs::Vector3Stamped YamlActionLoader::parseVector(YAML::Node yaml_object) {
	geometry_msgs::Vector3Stamped vector;
	if (yaml_object["frame"]) {
		vector.header.frame_id = yaml_object["frame"].as<std::string>();
	}
	if (yaml_object["position"]) {
		vector.vector.x = yaml_object["position"][0].as<double>();
		vector.vector.y = yaml_object["position"][1].as<double>();
		vector.vector.z = yaml_object["position"][2].as<double>();
	}
	return vector;
}

trajectory_msgs::JointTrajectory YamlActionLoader::parseJointTrajectories(YAML::Node yaml_object) {
	trajectory_msgs::JointTrajectory joint_trajectory;
	for (YAML::Node joint_name : yaml_object["joint_names"]) {
		joint_trajectory.joint_names.push_back(joint_name.as<std::string>());
	}
	for (YAML::Node knot : yaml_object["knots"]) {
		trajectory_msgs::JointTrajectoryPoint point;
		point.time_from_start = ros::Duration(knot["time"].as<double>());
		for (YAML::Node position : knot["positions"]) {
			point.positions.push_back(position.as<double>());
		}
		for (YAML::Node effort : knot["effort"]) {
			point.effort.push_back(effort.as<double>());
		}
		joint_trajectory.points.push_back(point);
	}
	return joint_trajectory;
}


//trajectory_msgs::MultiDOFJointTrajectory YamlActionLoader::parseMultiDOFTrajectory(std::string joint_name, YAML::Node trajectory_parameters) {
//	trajectory_msgs::MultiDOFJointTrajectory trajectory;
//	trajectory.header.frame_id = trajectory_parameters['frame'].as<std::string>();
//	trajectory.joint_names.push_back(joint_name);
//
//	for (YAML::Node knot : trajectory_parameters["knots"]) {
//		trajectory_msgs::MultiDOFJointTrajectoryPoint point;
//
//		ros::Duration duration(trajectory_parameters["time"].as<double>());
//		point.time_from_start = duration;
//
//		geometry_msgs::Transform transform;
//		transform.translation.x = knot['position'][0].as<double>();
//		transform.translation.y = knot['position'][1].as<double>();
//		transform.translation.z = knot['position'][2].as<double>();
//
//		if (knot["orientation"]) {
//			if (knot["orientation"].size() == 4) {
//				transform.rotation.x = knot["orientation"][0].as<double>();
//				transform.rotation.y = knot["orientation"][1].as<double>();
//				transform.rotation.z = knot["orientation"][2].as<double>();
//				transform.rotation.w = knot["orientation"][3].as<double>();
//			}
//			if (knot["orientation"].size() == 3) {
//				tf::Quaternion quaternion;
//				double roll  = knot["orientation"][0].as<double>();
//				double pitch = knot["orientation"][1].as<double>();
//				double yaw   = knot["orientation"][2].as<double>();
//				quaternion.setRPY(roll, pitch, yaw);
//				transform.rotation.x = quaternion.x();
//				transform.rotation.y = quaternion.y();
//				transform.rotation.z = quaternion.z();
//				transform.rotation.w = quaternion.w();
//			}
//		}
//		point.transforms.push_back(transform);
//		trajectory.points.push_back(point);
//	}
//
//	return trajectory;
//}

//trajectory_msgs::JointTrajectory YamlActionLoader::parseJointTrajectory(YAML::Node trajectory_parameters) {
//	trajectory_msgs::JointTrajectory trajectory;
//
//	for (YAML::Node joint_name : trajectory_parameters["joint_names"]) {
//		trajectory.joint_names.push_back(joint_name.as<std::string>());
//	}
//
//	for (YAML::Node knot : trajectory_parameters["knots"]) {
//		trajectory_msgs::JointTrajectoryPoint point;
//
//		ros::Duration duration(knot["time"].as<double>());
//		point.time_from_start = duration;
//
//		for (YAML::Node position : knot["positions"]) {
//			point.positions.push_back(position.as<double>());
//		}
//
//		trajectory.points.push_back(point);
//	}
//
//	return trajectory;
//}

void YamlActionLoader::adaptCoordinates(free_gait_msgs::ExecuteStepsGoal& goal, tf::Transform transform) {
	double yaw = tf::getYaw(transform.getRotation());
	tf::Vector3 z_axis(0.0, 0.0, 1.0);
	tf::Quaternion rotation;
	rotation.setRotation(z_axis, yaw);
	transform.setRotation(rotation);

	for (free_gait_msgs::Step& step : goal.steps) {
		for (free_gait_msgs::Footstep& footstep : step.footstep) {
			tf::Vector3 temp;
			tf::pointMsgToTF(footstep.target.point, temp);
			if (!temp.isZero()) {
				temp = transform(temp);
				footstep.target.point.x = temp.x();
				footstep.target.point.y = temp.y();
				footstep.target.point.z = temp.z();
			}
		}
	}

//	for (free_gait_msgs::Step& step : goal.steps) {
//		for (free_gait_msgs::SwingData& swing_data : step.swing_data) {
//			tf::Vector3 temp;
//			tf::pointMsgToTF(swing_data.profile.target.point, temp);
//			if (!temp.isZero()) {
//				temp = transform(temp);
//				swing_data.profile.target.point.x = temp.x();
//				swing_data.profile.target.point.y = temp.y();
//				swing_data.profile.target.point.z = temp.z();
//			}
//
//			for (trajectory_msgs::MultiDOFJointTrajectoryPoint& point : swing_data.foot_trajectory.points) {
//				tf::vector3MsgToTF(point.transforms[0].translation, temp);
//				temp = transform(temp);
//				point.transforms[0].translation.x = temp.x();
//				point.transforms[0].translation.y = temp.y();
//				point.transforms[0].translation.z = temp.z();
//			}
//		}
//
//		for (free_gait_msgs::BaseShiftData& base_shift_data : step.base_shift_data) {
//			tf::Pose temp;
//			tf::poseMsgToTF(base_shift_data.profile.target.pose, temp);
//			if (!temp.getOrigin().isZero() && !(temp.getRotation().length() == 0.0)) {
//				temp.mult(temp, transform);
//				base_shift_data.profile.target.pose.position.x = temp.getOrigin().x();
//				base_shift_data.profile.target.pose.position.y = temp.getOrigin().y();
//				base_shift_data.profile.target.pose.position.z = temp.getOrigin().z();
//				base_shift_data.profile.target.pose.orientation.x = temp.getRotation().x();
//				base_shift_data.profile.target.pose.orientation.y = temp.getRotation().y();
//				base_shift_data.profile.target.pose.orientation.z = temp.getRotation().z();
//				base_shift_data.profile.target.pose.orientation.w = temp.getRotation().w();
//			}
//
//			for (trajectory_msgs::MultiDOFJointTrajectoryPoint& point : base_shift_data.trajectory.points) {
//				tf::transformMsgToTF(point.transforms[0], temp);
//				temp.mult(temp, transform);
//				point.transforms[0].translation.x = temp.getOrigin().x();
//				point.transforms[0].translation.y = temp.getOrigin().y();
//				point.transforms[0].translation.z = temp.getOrigin().z();
//				point.transforms[0].rotation.x = temp.getRotation().x();
//				point.transforms[0].rotation.y = temp.getRotation().y();
//				point.transforms[0].rotation.z = temp.getRotation().z();
//				point.transforms[0].rotation.w = temp.getRotation().w();
//			}
//		}
//	}
}

} /* namespace free_gate */
