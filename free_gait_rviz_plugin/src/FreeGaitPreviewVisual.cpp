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
      stateBatchPtr_(NULL),
      endEffectorTrajectories_(sceneManager, frameNode_)
{
}

FreeGaitPreviewVisual::~FreeGaitPreviewVisual()
{
  sceneManager_->destroySceneNode(frameNode_);
}

void FreeGaitPreviewVisual::clear()
{
  stateBatchPtr_ = NULL;
  endEffectorTrajectories_.clear();
}

void FreeGaitPreviewVisual::setStateBatch(const free_gait::StateBatch& stateBatch)
{
  stateBatchPtr_ = &stateBatch;
}

void FreeGaitPreviewVisual::visualizeEndEffectorTrajectories(const float width, const Ogre::ColourValue& color)
{
  const size_t maxPointsPerLine = 500;
  endEffectorTrajectories_.setMaxPointsPerLine(maxPointsPerLine);
  const size_t nLines = stateBatchPtr_->getEndEffectorPositions().size()
      * std::ceil(stateBatchPtr_->getStates().size() / (double) maxPointsPerLine);
  endEffectorTrajectories_.setNumLines(nLines);
  std::cout << "ceil States: " << stateBatchPtr_->getStates().size() / maxPointsPerLine << std::endl;
  std::cout << "Num liens: " << nLines << std::endl;
  endEffectorTrajectories_.setLineWidth(width);
  endEffectorTrajectories_.setColor(color.r, color.g, color.b, color.a);
  bool firstLine = true;
  for (const auto& positions : stateBatchPtr_->getEndEffectorPositions()) {
    if (!firstLine) endEffectorTrajectories_.newLine();
    std::cout << "visualizeEndEffectorTrajectories new Line " << std::endl;
    free_gait::Position previousPosition;
    size_t i = 0;
    for (const auto& positionElement : positions) {
      if (i >= maxPointsPerLine - 1) {
        endEffectorTrajectories_.newLine();
        std::cout << "BABYY " << i << std::endl;
        i = 0;
      }
      const free_gait::Position position(positionElement.second);
//      if ((position - previousPosition).norm() < 0.01) continue;
      endEffectorTrajectories_.addPoint(Ogre::Vector3(position.x(), position.y(), position.z()));
      previousPosition = position;
      ++i;
//      std::cout << position << std::endl;
    }
    if (firstLine) firstLine = false;
  }
  std::cout << "FINISH" << std::endl;
}


} /* namespace free_gait_rviz_plugin */
