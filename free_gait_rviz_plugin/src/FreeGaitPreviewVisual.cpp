/*
 * FreeGaitPreviewVisual.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_rviz_plugin/FreeGaitPreviewVisual.hpp"

namespace free_gait_rviz_plugin {

FreeGaitPreviewVisual::FreeGaitPreviewVisual(Ogre::SceneManager* sceneManager,
                                             Ogre::SceneNode* parentNode)
    : sceneManager_(sceneManager),
      frameNode_(parentNode->createChildSceneNode()),
      stateBatchPtr_(NULL)
{
  // Visible by default.
  setEnabledModul(Modul::EndEffectorTargets, true);
  setEnabledModul(Modul::EndEffectorTrajectories, true);
}

FreeGaitPreviewVisual::~FreeGaitPreviewVisual()
{
  sceneManager_->destroySceneNode(frameNode_);
}

void FreeGaitPreviewVisual::clear()
{
  stateBatchPtr_ = NULL;
  hideEnabled();
}

void FreeGaitPreviewVisual::setStateBatch(const free_gait::StateBatch& stateBatch)
{
  stateBatchPtr_ = &stateBatch;
}

void FreeGaitPreviewVisual::setEnabledModul(Modul modul, bool enable)
{
  if (enable) {
    auto iterator = std::find(enabledModules_.begin(), enabledModules_.end(), modul);
    if (iterator != enabledModules_.end()) return;
    enabledModules_.push_back(modul);
  } else {
    enabledModules_.remove(modul);
  }
}

void FreeGaitPreviewVisual::showEnabled()
{
  ROS_DEBUG("FreeGaitPreviewVisual::showEnabled()");
  for (const auto& modul : enabledModules_) {
    switch (modul) {
      case Modul::EndEffectorTargets:
        showEndEffectorTargets();
        break;
      case Modul::EndEffectorTrajectories:
        showEndEffectorTrajectories();
        break;
      default:
        break;
    }
  }
}

void FreeGaitPreviewVisual::hideEnabled()
{
  for (const auto& modul : enabledModules_) {
    switch (modul) {
      case Modul::EndEffectorTargets:
        hideEndEffectorTargets();
        break;
      case Modul::EndEffectorTrajectories:
        hideEndEffectorTrajectories();
        break;
      default:
        break;
    }
  }
}

void FreeGaitPreviewVisual::showEndEffectorTargets(const float diameter, const Ogre::ColourValue& color)
{
  ROS_DEBUG("Rendering end effector targets.");
//  if (!stateBatchPtr_) return;
//  endEffectorTargets_.clear();
//  const size_t nEndEffectors(stateBatchPtr_->getEndEffectorTargets().size());
//
//  for (size_t i = 0; i < nEndEffectors; ++i) {
//    endEffectorTargets_.push_back(std::vector<std::unique_ptr<rviz::Shape>>());
//    // For each limb.
//    for (const auto& target : stateBatchPtr_->getEndEffectorTargets()[i]) {
//      // For each target.
//      const auto& targetPosition = target.second;
//      std::unique_ptr<rviz::Shape> shape(new rviz::Shape(rviz::Shape::Type::Sphere, sceneManager_, frameNode_));
//      shape->setPosition(Ogre::Vector3(targetPosition.x(), targetPosition.y(), targetPosition.z()));
//      shape->setColor(color);
//      shape->setScale(Ogre::Vector3(diameter));
//      endEffectorTargets_[i].push_back(std::move(shape));
//    }
//  }
}

void FreeGaitPreviewVisual::hideEndEffectorTargets()
{
//  endEffectorTargets_.clear();
}

void FreeGaitPreviewVisual::showEndEffectorTrajectories(const float width, const Ogre::ColourValue& color)
{
  ROS_DEBUG("Rendering end effector trajectories.");
  if (!stateBatchPtr_) return;

  // Define size.
  const size_t nEndEffectors(stateBatchPtr_->getEndEffectorPositions().size());
  const size_t nStates(stateBatchPtr_->getStates().size());

  // Cleanup.
  if (endEffectorTrajectories_.size() != nEndEffectors) {
    endEffectorTrajectories_.clear();
    for (size_t i = 0; i < nEndEffectors; ++i) {
      endEffectorTrajectories_.push_back(std::unique_ptr<rviz::BillboardLine>(new rviz::BillboardLine(sceneManager_, frameNode_)));
    }
  }

  for (size_t i = 0; i < nEndEffectors; ++i) {
    auto& trajectory = endEffectorTrajectories_[i];
    // For each foot trajectory.
    trajectory->clear();
    trajectory->setLineWidth(width);
    trajectory->setColor(color.r, color.g, color.b, color.a);
    trajectory->setNumLines(1);
    trajectory->setMaxPointsPerLine(nStates);

    free_gait::Position previousPosition(free_gait::Position::Zero());
    for (const auto& positionElement : stateBatchPtr_->getEndEffectorPositions()[i]) {
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

} /* namespace free_gait_rviz_plugin */
