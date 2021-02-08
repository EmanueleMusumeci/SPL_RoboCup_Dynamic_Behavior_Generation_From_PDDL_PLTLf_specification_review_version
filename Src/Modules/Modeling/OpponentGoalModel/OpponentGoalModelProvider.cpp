/**
 * @file Modules/Modeling/OpponentGoalModel/OpponentGoalModelProvider.cpp
 *
 * This module was used for debugging purposes but all the values are now loaded directly 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "OpponentGoalModelProvider.h"

OpponentGoalModelProvider::OpponentGoalModelProvider(){}

void OpponentGoalModelProvider::update(OpponentGoalModel& opponentGoalModel)
{
    opponentGoalModel.graphicalDebug = (GRAPHICAL_DEBUG==1 ? true : false);

    opponentGoalModel.useAreaDiscretization = (USE_AREA_DISCRETIZATION==1 ? true : false);

    opponentGoalModel.areaSizeMultiplicator = AREA_SIZE_MULTIPLICATOR;
    opponentGoalModel.goalTargetMinOffsetFromSideMultiplicator = GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR;
    opponentGoalModel.goalTargetDistanceThreshold = GOAL_TARGET_DISTANCE_THRESHOLD;
    opponentGoalModel.goalTargetObstacleInflation = GOAL_TARGET_OBSTACLE_INFLATION;
    opponentGoalModel.baseUtility = BASE_UTILITY;

    opponentGoalModel.goalTargetAreaMinSize = theBallSpecification.radius*AREA_SIZE_MULTIPLICATOR;
    opponentGoalModel.goalTargetMinOffsetFromSide = opponentGoalModel.goalTargetAreaMinSize * GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR;

    #ifdef TARGET_SIM
    if(opponentGoalModel.graphicalDebug)
    {
        opponentGoalModel.freeGoalTargetableAreas = theLibCheck.computeFreeAreas(opponentGoalModel.goalTargetAreaMinSize);
        opponentGoalModel.myGazeProjection = theLibCheck.projectGazeOntoOpponentGroundline();
        opponentGoalModel.shootASAPGoalTarget = theLibCheck.goalTarget(true);
        opponentGoalModel.utilityGoalTarget = theLibCheck.goalTarget(false);
        if(opponentGoalModel.freeGoalTargetableAreas.size()!=0)
        {
            opponentGoalModel.maxUtilityValue = opponentGoalModel.freeGoalTargetableAreas.front().value;
            opponentGoalModel.minUtilityValue = opponentGoalModel.freeGoalTargetableAreas.back().value;
        }
        else
        {
            opponentGoalModel.maxUtilityValue=0;
            opponentGoalModel.minUtilityValue=0;
        }
    }
    #endif
}

MAKE_MODULE(OpponentGoalModelProvider, modeling)