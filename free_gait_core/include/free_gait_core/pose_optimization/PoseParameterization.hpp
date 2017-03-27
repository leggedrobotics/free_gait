/*
 * PoseParameterization.hpp
 *
 *  Created on: Mar 22, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"

#include <numopt_common/Parameterization.hpp>

namespace free_gait {

class PoseParameterization : public numopt_common::Parameterization
{
 public:
  PoseParameterization();
  virtual ~PoseParameterization();

//  PoseParameterization(const PoseParameterization& other);

  numopt_common::Params& getParams();
  const numopt_common::Params& getParams() const;

  bool plus(numopt_common::Params& result, const numopt_common::Params& p,
            const numopt_common::Delta& dp) const;

  bool getTransformMatrixLocalToGlobal(numopt_common::SparseMatrix& matrix,
                                       const numopt_common::Params& params) const;

  bool getTransformMatrixGlobalToLocal(numopt_common::SparseMatrix& matrix,
                                       const numopt_common::Params& params) const;

  int getGlobalSize() const;
  static const size_t getGlobalSizeStatic();
  int getLocalSize() const;

  bool setRandom(numopt_common::Params& p) const;
  bool setIdentity(numopt_common::Params& p) const;

  Parameterization* clone() const;

  const Pose getPose() const;
  void setPose(const Pose& pose);
  const Position getPosition() const;
  const RotationQuaternion getOrientation() const;

 private:
  const static size_t nTransGlobal_ = 3;
  const static size_t nRotGlobal_ = 4;
  const static size_t nTransLocal_ = 3;
  const static size_t nRotLocal_ = 3;

  //! Global state vector with position and orientation.
  numopt_common::Params params_;
};

} /* namespace free_gait */
