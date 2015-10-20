/*
 * TypeDefs.hpp
 *
 *  Created on: Jun 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "kindr/poses/PoseEigen.hpp"
#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"

namespace free_gait {

typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
typedef kindr::poses::eigen_impl::TwistLinearVelocityLocalAngularVelocityPD Twist;

typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
typedef kindr::rotations::eigen_impl::RotationMatrixPD RotationMatrix;
typedef kindr::rotations::eigen_impl::EulerAnglesZyxPD EulerAnglesZyx;
typedef kindr::rotations::eigen_impl::RotationVectorPD RotationVector;

typedef kindr::phys_quant::eigen_impl::Position3D Position;

typedef kindr::phys_quant::eigen_impl::Velocity3D LinearVelocity;
typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;

typedef kindr::phys_quant::eigen_impl::Acceleration3D LinearAcceleration;
typedef kindr::phys_quant::eigen_impl::AngularAcceleration3D AngularAcceleration;

typedef kindr::phys_quant::eigen_impl::Force3D Force;
typedef kindr::phys_quant::eigen_impl::Torque3D Torque;

typedef kindr::phys_quant::eigen_impl::VectorTypeless3D Vector;

typedef kindr::phys_quant::eigen_impl::Position<double, Eigen::Dynamic> JointPositions;
typedef kindr::phys_quant::eigen_impl::Velocity<double, Eigen::Dynamic> JointVelocities;
typedef kindr::phys_quant::eigen_impl::Torque<double, Eigen::Dynamic> JointTorques;

} // namespace
