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
#include "Representations/Modeling/BallModel.h"
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
CARD(ApproachAndCarryCard,
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
  CALLS(WalkToCarry),
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(LogFloatParameter),
  CALLS(LogStringParameter),
  CALLS(KeyFrameArms),

  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
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
  //DONE 2) Switch to ApproachAndKick when in the kickable area
  //3) Sometimes the robot is stuck in the kick state: devise a smart way to find when to go back to walkToBall
  //4) IDEA add new state different by walkToBall that realigns the robot to the ball (for small movements) -> Two step realignment as proposed by Vincenzo

  //USING CFG PARAMETERS (FOUND IN FILE spqrnao2021/Config/Scenarios/Default/BehaviorControl/approachAndCarryCard.cfg).
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
    (Rangef) ballAlignmentRange,

    (float) ballOffsetY,
    (int) maxKickWaitTime,
    (float) goalKickThreshold,
    (float) nearGoalThreshold,

    (bool) DEBUG_MODE,
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

class ApproachAndCarryCard : public ApproachAndCarryCardBase
{

  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  bool kickRequested = false;
  bool targetChosen = false;
  Angle ballAlignThreshold = Angle::fromDegrees(ballAlignThreshold_degrees);

  float ballOffsetX = approachXRange.min + (approachXRange.max - approachXRange.min)/2;

  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f previousBallPosition;

  bool preconditions() const override
  {
    //std::cout << "Carrying" << '\n';
    if(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() > theFieldDimensions.xPosOpponentGroundline - 1300.0f)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  bool postconditions() const override
  {
    if(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() > theFieldDimensions.xPosOpponentGroundline - 1300.0f)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndCarry);

    initial_state(start)
    {
      transition
      {
        std::cout<<"APPROACH_AND_CARRY: start"<<std::endl;
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

        std::cout<<"turnToBall"<<std::endl;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theBallModel.estimate.position.angle()) < ballAlignThreshold)
          goto choose_target;

      }

      action
      {
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));

      }
    }

    /*common_transition{
      if(theLibCheck.distance(previousBallPosition, theFieldBall.positionOnField) > ballPositionChangedThreshold)
        //|| theLibCheck.distance(theBallCarrierModel.dynamicTarget.translation, chosenTarget) > targetPositionChangedThreshold
      {
        goto choose_target;
      }        
    }*/

    state(choose_target)
    {
      transition
      {
        std::cout<<"choose_target"<<std::endl;
        if(targetChosen)
        {
          targetChosen = false;
          std::cout<<"Target chosen"<<std::endl;
          if(DEBUG_MODE)
          {
            goto debug_state;
          }
          else
          {
            goto walkToBall;  
          }
        }
      }
      action
      {
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        goalTarget = theLibCheck.goalTarget(false);
        chosenTarget = theBallCarrierModel.dynamicTarget.translation;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        targetChosen = true;
      }
    }

    state(debug_state)
    {
      transition
      {}

      action
      {
        theStandSkill();
      }
    }

    state(walkToBall)
    {
      transition
      {
        std::cout<<"walkToBallFar"<<std::endl;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        float distance = theLibCheck.distance(theBallModel.estimate.position, theRobotPose);
          std::cout<<"theLibCheck.distance(theBallModel.estimate.position, theRobotPose): "<<theLibCheck.distance(theBallModel.estimate.position, theRobotPose)<<std::endl;
        if(approachXRange.isInside(theLibCheck.distance(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theRobotPose)))
        {
          std::cout<<"approachXRange.isInside(theLibCheck.distance(theBallModel.estimate.position, theRobotPose))"<<std::endl;
          if(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
          {
            std::cout<<"In kicking pose"<<std::endl;
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

        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        Vector2f ballPositionRelative = theBallModel.estimate.position;
        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        
        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, chosenTarget);
        std::cout<<"ballToTargetAngle: "<<ballToTargetAngle<<std::endl;
        
        float dynamicOffsetX = ballOffsetX * cos(pi + ballToTargetAngle);
        float dynamicOffsetY = ballOffsetX * sin(pi + ballToTargetAngle);
        std::cout<<"ballOffsetX: "<<ballOffsetX<<std::endl;
        std::cout<<"dynamicOffsetX: "<<dynamicOffsetX<<std::endl;
        std::cout<<"dynamicOffsetY: "<<dynamicOffsetY<<std::endl; 

        Pose2f approachPose = Pose2f(-ballToTargetAngle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
        Pose2f approachPoseRelative = theLibCheck.glob2RelWithAngle(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y()), ballPositionGlobal.x() - dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
        std::cout<<"approachPoseX: "<<approachPose.translation.x()<<std::endl;
        std::cout<<"approachPoseY: "<<approachPose.translation.y()<<std::endl;
        std::cout<<"approachPoseAngle: "<<Angle(approachPose.rotation).toDegrees()<<std::endl;

        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), approachPose);
      }
    }

    state(turnToTarget)
    {
      transition
      {
        std::cout<<"turnToTarget"<<std::endl;
        if(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          std::cout<<"ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees())"<<std::endl;
          goto walkToBall;
        }
      }
      action
      {

        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(1.f,1.f,1.f), Pose2f(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y())));
      }
    }

    state(kick)
    {
      std::cout<<"kick"<<std::endl;
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)){
          std::cout<<"!theFieldBall.ballWasSeen(ballNotSeenTimeout)"<<std::endl;
          goto searchForBall;
        }

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) > ballPositionChangedThreshold)
        {
          goto choose_target;
        }

        if(!(ballAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()) || ballAlignmentRange.isInside(theLibCheck.angleToBall)))
        {
          std::cout<<"REALIGN_TO_BALL"<<std::endl;
          goto walkToBall;
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"KICK TIMEOUT"<<std::endl;
          goto start;
        }
      }

      action
      {
        //double targetDistance =  (chosenTarget - theFieldBall.positionOnField).norm();
        // Since we are kicking, we don't want the ball to arrive just on the opponent goal line. So let's add 2 meters.

        //if (not kickRequested){
            //kickRequested = true;
            //std::string targetDistanceString = std::to_string(int(targetDistance/1000.f));
            //SystemCall::say("IN WALK KICKING TO DISTANCE");
            //SystemCall::say(targetDistanceString.c_str());
            //SystemCall::say("METERS");
        //}
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);
        /*if(theFieldBall.positionOnField.x()>goalKickThreshold) //If the ball is near enough to the goal to score with a single long kick
        {
          if(theFieldBall.positionOnField.x()>nearGoalThreshold) //If the ball is right in front of the goal use a strong kick
          {
            std::cout<<"A"<<std::endl;
            theKickSkill(false, distanceConfirmed, false);
          }
          else
          {
            if(theBallCarrierModel.isTargetOnGoal & !theBallCarrierModel.isFallbackPath) //else if the target is on the goal and it's not a fallback target
                                                                                          //(meaning the path planner considered this path free from obstacles)
                                                                                          //use a strong kick
            {
              std::cout<<"B"<<std::endl;
              theKickSkill(false, distanceConfirmed, false);
            }
            else //if the path is a fallback one OR the path goes around obstacles (therefore the target is not on the goal line but just a step along the path)
                 //use the InWalkKick
            {
              std::cout<<"C"<<std::endl;
              //theKickSkill(false, distanceConfirmed, false);
              theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x() - ballOffsetX, theBallModel.estimate.position.y() - ballOffsetY));
            }

          }
        }
        else
        {
          std::cout<<"D"<<std::endl;
          //theKickSkill(false, distanceConfirmed, false);
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x() - ballOffsetX, theBallModel.estimate.position.y() - ballOffsetY));
        }*/

        //InWalkKick per buttare a destra, target che sta leggermente a sinistra e viceversa (non più di 70° nel cfg)
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x() - ballOffsetX, theBallModel.estimate.position.y() - ballOffsetY));
       }
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
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
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

MAKE_CARD(ApproachAndCarryCard);
