/**
 * @file Modules/Modeling/OpponentGoalModel/OpponentGoalModelProvider.h
 *
 * This file implements a module that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

MODULE(OpponentGoalModelProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    PROVIDES(OpponentGoalModel),
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                      /** Show graphical debug in SimRobot */

      (bool) USE_AREA_DISCRETIZATION,                              /** Use free areas discretization + utility-based choice of target */ 

      (float) AREA_SIZE_MULTIPLICATOR,                             /** ball_radius * AREA_SIZE_MULTIPLICATOR is the min targetable area size in goalTarget */
      (float) GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR,      /** multiplicator for the minimum offset for target from left/right side of targetable area size in goalTarget */
      (float) GOAL_TARGET_DISTANCE_THRESHOLD,                      /** distance from opponent goal at which the utility-based targeting is deactivated in favour of the shootASAP mode */
      (float) GOAL_TARGET_OBSTACLE_INFLATION,                      /** multiplicator used to inflate the projection of obstacles based on their distance */
      (float) BASE_UTILITY,                                        /** Utility baseline to make sure that utility is always positive */
    }),
});

/**
 * @class OpponentGoalModelProvider
 * A module that provides the model of the opponent goal
 */
class OpponentGoalModelProvider: public OpponentGoalModelProviderBase
{
public:
  /** Constructor*/
  OpponentGoalModelProvider();

private:
  void update(OpponentGoalModel& opponentGoalModel) override;
};
