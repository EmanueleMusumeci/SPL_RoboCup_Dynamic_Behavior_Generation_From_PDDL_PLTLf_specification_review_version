/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.h
 *
 * This file implements a module that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"
#include <iostream>

MODULE(BallCarrierModelProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(LibPathPlanner),
    REQUIRES(FieldDimensions),
    REQUIRES(BallModel),
    REQUIRES(Role),
    REQUIRES(GameInfo),
    PROVIDES(BallCarrierModel),
    LOADS_PARAMETERS(
    {,
      (bool) GRAPHICAL_DEBUG,                                      /** Shows a graphical debug render in SimRobot */
      (float) OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR,            /** Fraction of 2*pi used as angle increments to compute segments of arc for going around obstacles */
    }),
});


/**
 * @class BallCarrierModelProvider
 * A module that provides the model of the opponent goal
 */
class BallCarrierModelProvider: public BallCarrierModelProviderBase
{
public:
  /** Constructor*/
  BallCarrierModelProvider();

private:
  void update(BallCarrierModel& ballCarrierModel);
};
