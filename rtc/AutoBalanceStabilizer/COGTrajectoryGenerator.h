// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef COGTRAJECTORYGENERATOR_H
#define COGTRAJECTORYGENERATOR_H

#include <vector>
#include <memory>
#include <hrpUtil/EigenTypes.h>
#include "PreviewController.h"

namespace hrp {

class COGTrajectoryGenerator
{
  public:
    enum CogCalculationType : size_t { PREVIEW_CONTROL};

  private:
    hrp::Vector3 cog     = hrp::Vector3::Zero();
    hrp::Vector3 cog_vel = hrp::Vector3::Zero();
    hrp::Vector3 cog_acc = hrp::Vector3::Zero();

    CogCalculationType calculation_type = PREVIEW_CONTROL;
    std::unique_ptr<ExtendedPreviewController> preview_controller;

  public:
    COGTrajectoryGenerator() {}

    const hrp::Vector3& getCog() const { return cog; }
    const hrp::Vector3& getCogVel() const { return cog_vel; }
    const hrp::Vector3& getCogAcc() const { return cog_acc; }

    void initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp);
    void calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list);
};

}

#endif // COGTRAJECTORYGENERATOR_H
