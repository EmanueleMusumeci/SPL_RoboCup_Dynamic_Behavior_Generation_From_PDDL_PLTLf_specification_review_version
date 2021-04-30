/**
 * @file ApproachAndCarryCard.cpp
 *
 * This file implements a behavior to carry the ball forward in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci (based on Emanuele Antonioni's basic approacher behavior structure)
 * SORRY FOR THE S****Y CODE, DIDN'T HAVE MUCH TIME, WILL FIX IT
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(C2CarryTheBallCard,
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
  //CALLS(WalkToTargetPathPlannerFixedAngle),
  CALLS(WalkToCarry),
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(LogFloatParameter),
  CALLS(LogStringParameter),

  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(BallCarrierModel),
  REQUIRES(LibCheck),
  REQUIRES(LibPathPlanner),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  USES(BehaviorStatus),

  REQUIRES(GameInfo),

  //TODO:
  //1) Arms behind the back when near obstacles
  //2) Switch to ApproachAndKick when in the kickable area
  //3) Sometimes the robot is stuck in the kick state: devise a smart way to find when to go back to walkToBall_far
  //4) IDEA add new state different by walkToBall_far that realigns the robot to the ball (for small movements)

  //USING CFG PARAMETERS (FOUND IN FILE spqrnao2021/Config/Scenarios/Default/BehaviorControl/approachAndKickCard.cfg).
  //IF THERE IS ANY PROBLEM COMMENT THE FIRST BLOCK OF PARAMETERS AND UNCOMMENT THE SECOND

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    
    (int) initialWaitTime,
    (int) ballNotSeenTimeout,
    (int) changeTargetTimeout,

    (float) ballAlignThreshold_degrees,

    (float) ballPositionChangedThreshold,
    (float) targetPositionChangedThreshold,

    (Rangef) approachXRange,
    (Rangef) approachYRange,
    (Rangef) smallApproachYRange,
    (Rangef) smallApproachXRange,
    (Rangef) ballAlignmentRange,
    (float) ballOffsetY,
    (Rangef) ballOffsetYRange,
    (int) minKickWaitTime,
    (int) maxKickWaitTime,
    (float) approachTreshold,
    (bool) debugText,
    (float) goalKickThreshold,
    (float) nearGoalThreshold,

    (float) 
  }),

  /*DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(180.f) ballOffsetX,
    (Rangef)({170.f, 190.f}) ballOffsetXRange,
    (Rangef)({-350.f, 350.f}) approachYRange,
    (Rangef)({-150.f, 150.f}) smallApproachYRange,
    (Rangef)({150.f, 300.f}) smallApproachXRange,
    (float)(-75.f) ballOffsetY,
    (Rangef)({-85.f, -65.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (float)(1000.f) approachTreshold,
    (bool)(true) debugText,
    (float)(1000) goalKickThreshold,
    (float)(2500) nearGoalThreshold,
  }),*/
});

class C2CarryTheBallCard : public C2CarryTheBallCardBase
{
  bool kickRequested = false;
  bool targetChosen = false;
  Angle ballAlignThreshold = Angle::fromDegrees(ballAlignThreshold_degrees);

  float ballOffsetX = approachXRange.min + (approachXRange.max - approachXRange.min)/2;

  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f previousBallPosition;

  bool preconditions() const override
  {
    bool passingArea = theLibCheck.C2PassingArea();
    bool ownField = theLibCheck.C2OwnField();
    float distance = theLibCheck.distance(theRobotPose.translation, theFieldBall.positionOnField);

    if (!ownField) return false;
    else return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndCarry);

    initial_state(start)
    {
      transition
      {
        //std::cout<<"start"<<std::endl;
        if(state_time > initialWaitTime)
          goto choose_target;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {

        //std::cout<<"turnToBall"<<std::endl;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto choose_target;

      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));

      }
    }
    state(choose_target)
    {
      transition
      {
        //std::cout<<"choose_target"<<std::endl;
        if(targetChosen)
        {
          targetChosen = false;
          //std::cout<<"Target chosen"<<std::endl;
          goto walkToBall;
        }
      }
      action
      {
        Pose2f targetPose = theLibCheck.C2EvaluateTarget(1);

        chosenTarget = targetPose.translation;
        previousBallPosition = theFieldBall.positionOnField;
        targetChosen = true;
      }
    }

    state(walkToBall)
    {
      transition
      {
        //std::cout<<"walkToBallFar"<<std::endl;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        float distance = theLibCheck.distance(theFieldBall.positionRelative, theRobotPose);
        if(approachXRange.isInside(theLibCheck.distance(theFieldBall.positionOnField, theRobotPose)))
        {
          if(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
          {
            goto kick;
          }
        }
        
        if(state_time > changeTargetTimeout)
        {
          goto choose_target;
        }
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));

        Pose2f ballPose = Pose2f(theFieldBall.positionOnField);

        float ballToTargetAngle = theLibCheck.C2AngleBetween(chosenTarget, ballPose, false);
        
        Pose2f offset = theLibCheck.C2EvaluateApproach(chosenTarget);
        float dynamicOffsetX = offset.translation.x();
        float dynamicOffsetY = offset.translation.y();

        Pose2f approachPose = Pose2f(ballToTargetAngle, theFieldBall.positionOnField.x() + dynamicOffsetX, theFieldBall.positionOnField.y() + dynamicOffsetY);
        Pose2f approachPoseRelative = theLibCheck.glob2Rel(theFieldBall.positionOnField.x() - dynamicOffsetX, theFieldBall.positionOnField.y() - dynamicOffsetY);
      
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), approachPose);
      }
    }

    state(turnToTarget)
    {
      transition
      {
        //std::cout<<"turnToTarget"<<std::endl;
        if(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          //std::cout<<"ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees())"<<std::endl;
          goto walkToBall;
        }
      }
      action
      {
        theWalkToTargetSkill(Pose2f(1.f,1.f,1.f), Pose2f(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())));
      }
    }

    state(kick)
    {
      //std::cout<<"Kick"<<std::endl;
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)){
          //std::cout<<"!theFieldBall.ballWasSeen(ballNotSeenTimeout)"<<std::endl;
          goto searchForBall;
        }

        if(theLibCheck.distance(previousBallPosition, theFieldBall.positionOnField) > ballPositionChangedThreshold)
        {
          goto choose_target;
        }

        if(!(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()) || ballAlignmentRange.isInside(theLibCheck.angleToBall)))
        {
          //std::cout<<"REALIGN_TO_BALL"<<std::endl;
          goto walkToBall;
        }

        if(state_time > maxKickWaitTime) //|| (state_time > minKickWaitTime && theKickSkill.isDone())){
        {  
          //std::cout<<"KICK TIMEOUT"<<std::endl;
          goto start;
        }
      }

      action
      {
        double targetDistance =  (chosenTarget - theFieldBall.positionOnField).norm();

        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y()));
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);}
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
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

MAKE_CARD(C2CarryTheBallCard);