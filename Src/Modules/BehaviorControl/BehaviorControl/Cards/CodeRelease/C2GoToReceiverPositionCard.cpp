/**
 * @file ApproachAndKickCard.cpp
 *
 * This file implements a behavior for approaching the ball and kick it to a given target.
 *
 * @author Emanuele Antonioni
 */
 
// TODO: Scrivere il behaviour

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/BHMath.h"

CARD(C2GoToReceiverPositionCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToApproach),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    }),
});

class C2GoToReceiverPositionCard : public C2GoToReceiverPositionCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    theActivitySkill(BehaviorStatus::defender);

    initial_state(start)
    {
      transition
      {
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
        
        if(std::abs(theRobotPose.rotation.toDegrees()) > 15 ){
            goto walkToPoint_near;
        }
        if(theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -500.f)) > positionTh){
            goto walkToPoint_far;
        }
          
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theStandSkill();
      }
    }

    
    state(walkToPoint_far)
    {
      transition
      {
        
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
        

        if(theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -500.f)) <= smallPositionTh){
            goto walkToPoint_near;
        }
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f));
        
      }
    }

    state(walkToPoint_near){
      transition{
        
        if(theRole.role == Role::searcher_1 || theRole.role == Role::searcher_2 || theRole.role == Role::searcher_4)
          goto searchForBall;
        
        if(theLibCheck.distance(theRobotPose, Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -500.f)) <= 100){
            if(std::abs(theRobotPose.rotation.toDegrees()) < 10 ){
                goto start;
            }
            
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        theWalkToTargetPathPlannerStraightSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f));
      }
    }
    
    
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(C2GoToReceiverPositionCard);



