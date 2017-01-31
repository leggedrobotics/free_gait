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
  setEnabledModul(Modul::EndEffectorTrajectories, true);
  setEnabledModul(Modul::Footsteps, true);
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
    auto iterator = std::find(enabledModuls_.begin(), enabledModuls_.end(), modul);
    if (iterator != enabledModuls_.end()) return;
    enabledModuls_.push_back(modul);
  } else {
    enabledModuls_.remove(modul);
  }
}

void FreeGaitPreviewVisual::showEnabled()
{
  ROS_DEBUG("FreeGaitPreviewVisual::showEnabled()");
  for (const auto& modul : enabledModuls_) {
    switch (modul) {
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
  for (const auto& modul : enabledModuls_) {
    switch (modul) {
      case Modul::EndEffectorTrajectories:
        hideEndEffectorTrajectories();
        break;
      default:
        break;
    }
  }
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
