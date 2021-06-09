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

#include "Representations/HRI/HRIController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(IdleCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(Kick),

  REQUIRES(HRIController),
  REQUIRES(RobotPose),

  USES(BehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (int) initialWaitTime,
  }),
  
});

class IdleCard : public IdleCardBase
{

  bool preconditions() const override
  {
    std::cout<<"theHRIController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theHRIController.getCurrentActionType())<<std::endl;
    //return theHRIController.getCurrentActionType() == HRI::ActionType::Idle;
    return true;
  }

  bool postconditions() const override
  {
    return theHRIController.getCurrentActionType() != HRI::ActionType::Idle;
  }

  option
  {

    initial_state(start)
    {
      std::cout<<"REACH_POSITION: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto idle_state;
      }

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookForwardSkill();
        theStandSkill();
      }
    }
    
    state(idle_state)
    {
      transition
      {}

      action
      {
        theActivitySkill(BehaviorStatus::idle);
        theLookForwardSkill();
        theStandSkill();
      }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(IdleCard);
