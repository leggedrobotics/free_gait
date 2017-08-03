/*
 * FreeGaitPreviewVisual.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_rviz_plugin/FreeGaitPreviewVisual.hpp"

#include <free_gait_ros/RosVisualization.hpp>

#include <rviz/default_plugin/markers/triangle_list_marker.h>

namespace free_gait_rviz_plugin {

FreeGaitPreviewVisual::FreeGaitPreviewVisual(rviz::DisplayContext* context, Ogre::SceneNode* parentNode)
    : context_(context),
      frameNode_(parentNode->createChildSceneNode()),
      stateBatchPtr_(NULL)
{
  // Visible by default.
  setEnabledModul(Modul::EndEffectorTargets, true);
  setEnabledModul(Modul::EndEffectorTrajectories, true);
  setEnabledModul(Modul::Stances, true);
}

FreeGaitPreviewVisual::~FreeGaitPreviewVisual()
{
  context_->getSceneManager()->destroySceneNode(frameNode_);
}

void FreeGaitPreviewVisual::clear()
{
  stateBatchPtr_ = NULL;

  for (const auto& modul : enabledModules_) {
    switch (modul) {
      case Modul::EndEffectorTargets:
        hideEndEffectorTargets();
        break;
      case Modul::EndEffectorTrajectories:
        hideEndEffectorTrajectories();
        break;
      case Modul::Stances:
        hideStances();
        break;
      default:
        break;
    }
  }
}

void FreeGaitPreviewVisual::setStateBatch(const free_gait::StateBatch& stateBatch)
{
  stateBatchPtr_ = &stateBatch;
  modulesToEnable_ = enabledModules_;
}

void FreeGaitPreviewVisual::setEnabledModul(Modul modul, bool enable)
{
  if (enable) {
    modulesToEnable_.push_back(modul);
    auto iterator = std::find(enabledModules_.begin(), enabledModules_.end(), modul);
    if (iterator != enabledModules_.end()) return;
    enabledModules_.push_back(modul);
  } else {
    modulesToDisable_.push_back(modul);
    enabledModules_.remove(modul);
  }
}

void FreeGaitPreviewVisual::showEnabled()
{
  modulesToEnable_ = enabledModules_;
}

void FreeGaitPreviewVisual::hideEnabled()
{
  modulesToDisable_ = enabledModules_;
}

void FreeGaitPreviewVisual::update()
{
  for (const auto& modul : modulesToDisable_) {
    switch (modul) {
      case Modul::EndEffectorTargets:
        hideEndEffectorTargets();
        break;
      case Modul::EndEffectorTrajectories:
        hideEndEffectorTrajectories();
        break;
      case Modul::Stances:
        hideStances();
        break;
      default:
        break;
    }
  }
  modulesToDisable_.clear();

  for (const auto& modul : modulesToEnable_) {
    switch (modul) {
      case Modul::EndEffectorTargets:
        showEndEffectorTargets();
        break;
      case Modul::EndEffectorTrajectories:
        showEndEffectorTrajectories();
        break;
      case Modul::Stances:
        showStances();
        break;
      default:
        break;
    }
  }
  modulesToEnable_.clear();
}

void FreeGaitPreviewVisual::showEndEffectorTargets(const float diameter, const Ogre::ColourValue& color)
{
  ROS_DEBUG("Rendering end effector targets.");
  if (!stateBatchPtr_) return;

  endEffectorTargets_.clear();
  const auto targets = stateBatchPtr_->getEndEffectorTargets();
  const size_t nEndEffectors(targets.size());

  for (size_t i = 0; i < nEndEffectors; ++i) {
    endEffectorTargets_.push_back(std::vector<std::unique_ptr<rviz::Shape>>());
    // For each limb.
    for (const auto& target : targets[i]) {
      // For each target.
      const auto& targetPosition = target.second;
      endEffectorTargets_[i].push_back(
          std::unique_ptr<rviz::Shape>(new rviz::Shape(rviz::Shape::Type::Sphere, context_->getSceneManager(), frameNode_)));
      auto& shape = endEffectorTargets_[i].back();
      shape->setPosition(Ogre::Vector3(targetPosition.x(), targetPosition.y(), targetPosition.z()));
      shape->setColor(color);
      shape->setScale(Ogre::Vector3(diameter));
    }
  }
}

void FreeGaitPreviewVisual::hideEndEffectorTargets()
{
  endEffectorTargets_.clear();
}

void FreeGaitPreviewVisual::showEndEffectorTrajectories(const float width, const Ogre::ColourValue& color)
{
  ROS_DEBUG("Rendering end effector trajectories.");
  if (!stateBatchPtr_) return;

  const auto positions = stateBatchPtr_->getEndEffectorPositions();

  // Define size.
  const size_t nEndEffectors(positions.size());
  const size_t nStates(stateBatchPtr_->getStates().size());

  // Cleanup.
  if (endEffectorTrajectories_.size() != nEndEffectors) {
    endEffectorTrajectories_.clear();
    for (size_t i = 0; i < nEndEffectors; ++i) {
      endEffectorTrajectories_.push_back(std::unique_ptr<rviz::BillboardLine>(new rviz::BillboardLine(context_->getSceneManager(), frameNode_)));
    }
  }

  // Render.
  for (size_t i = 0; i < nEndEffectors; ++i) {
    auto& trajectory = endEffectorTrajectories_[i];
    // For each foot trajectory.
    trajectory->clear();
    trajectory->setLineWidth(width);
    trajectory->setColor(color.r, color.g, color.b, color.a);
    trajectory->setNumLines(1);
    trajectory->setMaxPointsPerLine(nStates);

    free_gait::Position previousPosition(free_gait::Position::Zero());
    for (const auto& positionElement : positions[i]) {
      const auto& position = positionElement.second;
      if ((position - previousPosition).norm() < 0.01) continue; // TODO Set as parameter.
      previousPosition = position;
      const auto point = Ogre::Vector3(position.x(), position.y(), position.z());
      trajectory->addPoint(point);
    }
  }
}

void FreeGaitPreviewVisual::hideEndEffectorTrajectories()
{
  for (auto& trajectory : endEffectorTrajectories_) {
    trajectory->clear();
  }
}

void FreeGaitPreviewVisual::showStances(const float alpha)
{
  ROS_DEBUG("Rendering stances.");
  if (!stateBatchPtr_) return;
  stancesMarker_.clear();

  for (const auto& stance : stateBatchPtr_->getStances()) {
    if (stance.second.empty()) continue;
    std_msgs::ColorRGBA color;
    getRainbowColor(stateBatchPtr_->getStartTime(), stateBatchPtr_->getEndTime(), stance.first, color);
    color.a = alpha;
    visualization_msgs::Marker marker = free_gait::RosVisualization::getStanceMarker(stance.second, "odom", // TODO Use adapter?
                                                                                     color);
    stancesMarker_.push_back(std::unique_ptr<rviz::TriangleListMarker>(new rviz::TriangleListMarker(nullptr, context_, frameNode_)));
    stancesMarker_.back()->setMessage(marker);
  }
}

void FreeGaitPreviewVisual::hideStances()
{
  stancesMarker_.clear();
}

void FreeGaitPreviewVisual::getRainbowColor(const double min, const double max, const double value, std_msgs::ColorRGBA& color)
{
  double adaptedValue = (value - min) / (max - min);

  // From rviz/src/rviz/default_plugin/point_cloud_transformers.cpp:
  adaptedValue = std::min(adaptedValue, 1.0);
  adaptedValue = std::max(adaptedValue, 0.0);

  float h = adaptedValue * 5.0 + 1.0;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1)) f = 1 - f; // If i is even.
  float n = 1 - f;
  if      (i <= 1) color.r = n, color.g = 0, color.b = 1;
  else if (i == 2) color.r = 0, color.g = n, color.b = 1;
  else if (i == 3) color.r = 0, color.g = 1, color.b = n;
  else if (i == 4) color.r = n, color.g = 1, color.b = 0;
  else if (i >= 5) color.r = 1, color.g = n, color.b = 0;
}

} /* namespace free_gait_rviz_plugin */
