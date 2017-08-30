/*
 * FreeGaitPreviewVisual.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <OGRE/Ogre.h>
#include <vector>
#include <memory>

namespace free_gait_rviz_plugin {

class FreeGaitPreviewVisual
{
 public:
  enum class Modul {
    EndEffectorTargets,
    SurfaceNormals,
    EndEffectorTrajectories,
    Stances
  };

  FreeGaitPreviewVisual(rviz::DisplayContext* context, Ogre::SceneNode* parentNode);
  virtual ~FreeGaitPreviewVisual();

  void clear();
  void setStateBatch(const free_gait::StateBatch& stateBatch);
  void setEnabledModul(Modul modul, bool enable);
  void showEnabled();
  void hideEnabled();
  void update();

 protected:
  void showEndEffectorTargets(const float diameter = 0.04,
                              const Ogre::ColourValue& color = Ogre::ColourValue(1, 0, 0, 1));
  void hideEndEffectorTargets();

  void showSurfaceNormals(const float diameter = 0.003, const float length = 0.07,
                          const Ogre::ColourValue& color = Ogre::ColourValue(1, 0, 0, 1));
  void hideSurfaceNormals();

  void showEndEffectorTrajectories(const float width = 0.01,
                                   const Ogre::ColourValue& color = Ogre::ColourValue(1, 0, 0, 1));
  void hideEndEffectorTrajectories();

  void showStances(const float alpha = 0.3);
  void hideStances();

  static void getRainbowColor(const double min, const double max, const double value, std_msgs::ColorRGBA& color);

 private:
  rviz::DisplayContext* context_;
  Ogre::SceneNode* frameNode_;
  const free_gait::StateBatch* stateBatchPtr_;
  std::list<Modul> modulesToEnable_;
  std::list<Modul> modulesToDisable_;
  std::list<Modul> enabledModules_;
  std::vector<std::vector<std::unique_ptr<rviz::Shape>>> endEffectorTargets_;
  std::vector<std::vector<std::unique_ptr<rviz::Arrow>>> surfaceNormals_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> endEffectorTrajectories_;
  std::vector<std::unique_ptr<rviz::MarkerBase>> stancesMarker_;
};

} /* namespace free_gait_rviz_plugin */
