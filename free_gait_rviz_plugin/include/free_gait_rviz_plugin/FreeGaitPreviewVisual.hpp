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
  enum class Modul {
    EndEffectorTrajectories,
    Footsteps
  };

  FreeGaitPreviewVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
  virtual ~FreeGaitPreviewVisual();

  void clear();
  void setStateBatch(const free_gait::StateBatch& stateBatch);
  void setEnabledModul(Modul modul, bool enable);
  void showEnabled();
  void hideEnabled();


  void showEndEffectorTrajectories(const float width = 0.01, const Ogre::ColourValue& color =
                                            Ogre::ColourValue(1, 0, 0, 1));
  void hideEndEffectorTrajectories();

 private:
  Ogre::SceneManager* sceneManager_;
  Ogre::SceneNode* frameNode_;
  const free_gait::StateBatch* stateBatchPtr_;
  std::list<Modul> enabledModuls_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> endEffectorTrajectories_;
};

} /* namespace free_gait_rviz_plugin */
