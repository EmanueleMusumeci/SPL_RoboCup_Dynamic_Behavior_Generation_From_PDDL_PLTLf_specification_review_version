/**
 * @file C1ApproachAndCarryWithTwoStepRealignmentCard.cpp
 *
 * This file implements a behavior to carry the ball forward in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci (based on Emanuele Antonioni's basic approacher behavior structure)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/HRI/HRIController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(ReachPositionCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkToTargetPathPlanner),
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(KeyFrameArms),

  REQUIRES(LibCheck),
  REQUIRES(RobotPose),

  REQUIRES(HRIController),

  USES(BehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    
    (int) initialWaitTime,
    (int) realignmentTimeout,

    //(Rangef) attemptRelocalizationEveryTimeRange,

    (Rangef) smallAlignmentRange,

    (bool) DEBUG_MODE,
  }),
});

class ReachPositionCard : public ReachPositionCardBase
{

  bool preconditions() const override
  {
    std::cout<<"theHRIController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theHRIController.getCurrentActionType())<<std::endl;
    return (theHRIController.getCurrentActionType() == HRI::ActionType::ReachPosition || theHRIController.getCurrentActionType() == HRI::ActionType::ReachBall)
          &&
          theLibCheck.distance(theRobotPose, theHRIController.currentRobotDestination) > theHRIController.reachPositionDistanceThreshold;
  }

  bool postconditions() const override
  {
    return (theHRIController.getCurrentActionType() != HRI::ActionType::ReachPosition && theHRIController.getCurrentActionType() != HRI::ActionType::ReachBall)
          ||
          theLibCheck.distance(theRobotPose, theHRIController.currentRobotDestination) <= theHRIController.reachPositionDistanceThreshold;
  }

  option
  {

    initial_state(start)
    {
      std::cout<<"REACH_POSITION: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto walkToPosition;
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_position);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(debug_state)
    {
      transition
      {}

      action
      {
        theActivitySkill(BehaviorStatus::debug_standing);
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(walkToPosition)
    {
      //std::cout<<"walk"<<std::endl;
      transition
      {
           if(theLibCheck.distance(theRobotPose, theHRIController.currentRobotDestination) <= theHRIController.reachPositionDistanceThreshold)
            goto stand;
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_position);
        theLookAtPointSkill(Vector3f(theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10.f));
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theHRIController.currentRobotDestination);
      }
    }

    state(stand)
    {
      //std::cout<<"walk"<<std::endl;
      transition
      {}

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookForwardSkill();
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theHRIController.currentRobotDestination);
      }
    }

  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(ReachPositionCard);
