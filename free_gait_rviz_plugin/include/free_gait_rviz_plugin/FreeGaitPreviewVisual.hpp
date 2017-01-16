/*
 * FreeGaitPreviewVisual.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// Free Gait
#include <free_gait_core/free_gait_core.hpp>

// RViz
#include <rviz/ogre_helpers/billboard_line.h>

// OGRE
#include <OGRE/Ogre.h>

// STD
#include <vector>
#include <memory>

namespace free_gait_rviz_plugin {

class FreeGaitPreviewVisual
{
 public:
  FreeGaitPreviewVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
  virtual ~FreeGaitPreviewVisual();

  void clear();
  void setStateBatch(const free_gait::StateBatch& stateBatch);

  void visualizeEndEffectorTrajectories(const float width, const Ogre::ColourValue& color);

 private:
  Ogre::SceneManager* sceneManager_;
  Ogre::SceneNode* frameNode_;
  const free_gait::StateBatch* stateBatchPtr_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> endEffectorTrajectories_;
};

} /* namespace free_gait_rviz_plugin */
