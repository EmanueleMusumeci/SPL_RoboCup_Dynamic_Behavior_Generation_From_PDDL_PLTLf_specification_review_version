/**
 * @file C1ApproachAndCarryToGoalWithTwoStepRealignmentCard.cpp
 *
 * This file implements a behavior to carry the ball forward in the field, avoiding obstacles.
 *
 * @author Emanuele Musumeci (based on Emanuele Antonioni's basic approacher behavior structure)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/HRI/HRIController.h"

#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(ApproachAndCarryToGoalWithTwoStepRealignmentCard,
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
  CALLS(GoalTarget),
  CALLS(SetTarget),
  CALLS(KeyFrameArms),

  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(BallCarrierModel),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(ObstacleModel),

  REQUIRES(HRIController),

  USES(BehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    
    (int) initialWaitTime,
    (int) ballNotSeenTimeout,
    (int) changeTargetTimeout,
    (int) realignmentTimeout,

    (Rangef) attemptRelocalizationEveryTimeRange,

    (float) distanceFromGroundLineCardChangeThreshold,

    (float) ballPositionChangedThreshold,
    (float) targetPositionChangedThreshold,

    (Rangef) approachXRange,

    (Rangef) approachYRange,

    //Two ball alignment angle ranges: the smaller one is used as an escape condition when aligning the robot to the ball (we should be more precise), 
    //the bigger one is used instead to decide when to realign (so we give more room to small errors in alignment) 
    (Rangef) smallBallAlignmentRange,
    (Rangef) ballAlignmentRange,

    (float) nearbyObstaclesRadius, //Radius inside which an obstacle is considered "nearby"

    (int) maxKickWaitTime,

    (bool) DEBUG_MODE,
  }),
});

class ApproachAndCarryToGoalWithTwoStepRealignmentCard : public ApproachAndCarryToGoalWithTwoStepRealignmentCardBase
{

  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  bool kickRequested = false;
  bool targetChosen = false;

  float ballOffsetX = approachXRange.min + (approachXRange.max - approachXRange.min)/2;
  //float smallBallOffsetX = approachXRange.min + (approachXRange.max - approachXRange.min)/3;
  float smallBallOffsetX = ballOffsetX;

  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f previousBallPosition;
  int previousNearbyObstaclesCount;

  bool preconditions() const override
  {
    std::cout<<"theHRIController.getCurrentActionType(): "<<TypeRegistry::getEnumName(theHRIController.getCurrentActionType())<<std::endl;
    return theHRIController.getCurrentActionType() == HRI::ActionType::CarryAndKickToGoal
                &&
                (theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() <= theFieldDimensions.xPosOpponentGroundline - distanceFromGroundLineCardChangeThreshold
                ||
                (theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() > theFieldDimensions.xPosOpponentGroundline - distanceFromGroundLineCardChangeThreshold
                    &&
                    !theBallCarrierModel.isTargetOnGoal));           
    ;
  }

  bool postconditions() const override
  {
    return theHRIController.getCurrentActionType() != HRI::ActionType::CarryAndKickToGoal
           ||
           (theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() > theFieldDimensions.xPosOpponentGroundline - distanceFromGroundLineCardChangeThreshold
                && 
                theBallCarrierModel.isTargetOnGoal);
  }

  option
  {

    /*common_transition
    {
      if(theRobotPose.translation.x() > theFieldDimensions.centerCircleRadius 
      && attemptRelocalizationEveryTimeRange.isInside(option_time % (int)((attemptRelocalizationEveryTimeRange.max - attemptRelocalizationEveryTimeRange.min)/2.f)))
      {
        goto attemptRelocalization;
      }
    }*/

    initial_state(start)
    {
      std::cout<<"APPROACH_AND_CARRY_WITH_TWO_STEP_REALIGNMENT: start"<<std::endl;
      transition
      {
        if(state_time > initialWaitTime)
          goto choose_target;
      }

      action
      {
        theActivitySkill(BehaviorStatus::approach_and_carry_start);
        
        theLookForwardSkill();
        theStandSkill();
      }
    }

    /*state(attemptRelocalization)
    {
      transition
      {
        
      }
      action
      {

      }
    }*/

    state(turnToBall)
    {
      std::cout<<"turnToBall"<<std::endl;
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        
        if(//approachXRange.isInside(theBallModel.estimate.position.x())
        //&& approachYRange.isInside(theBallModel.estimate.position.y()) &&
        smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          goto choose_target;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));

      }
    }

    state(choose_target)
    {
      std::cout<<"choose_target"<<std::endl;
      transition
      {

        if(DEBUG_MODE)
        {
          goto debug_state;
        }
        else
        {
          goto walkToBall;  
        }

        if(targetChosen)
        {
          targetChosen = false;
          goto walkToBall;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::choosing_target);

        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);

        goalTarget = theLibCheck.goalTarget(false, true);
        chosenTarget = theBallCarrierModel.dynamicTarget.translation;
        chosenTarget = theHRIController.currentBallDestination;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        previousNearbyObstaclesCount = countNearbyObstacles(nearbyObstaclesRadius);
        targetChosen = true;

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
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

        theStandSkill();
      }
    }

    state(walkToBall)
    {
      //std::cout<<"walk"<<std::endl;
      transition
      {

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }

        //if the LOCAL ball is inside the approach on the x axis 
        //AND the LOCAL ball is inside the approach range on the y axis 
        //AND the robot is aligned with the target
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(theBallModel.estimate.position.y())
        && smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          goto kick;
        }
        
        //NEWLY ADDED
        if(theBallModel.estimate.position.norm() < approachXRange.max
        && approachXRange.isInside(theBallModel.estimate.position.x())
        && !approachYRange.isInside(theBallModel.estimate.position.y()))
        {
          //Case B: Robot is in the X range but not in the Y range
          std::cout<<"kick: Case B"<<std::endl;
          goto secondStepAlignment;
        }

        if(state_time > changeTargetTimeout)
        {
          std::cout<<"walkToBall: change target timeout -> choose_target"<<std::endl;
          goto choose_target;
        }

        if(countNearbyObstacles(nearbyObstaclesRadius) > previousNearbyObstaclesCount)
        {
          std::cout<<"walkToBall: numberOfNearbyObstaclesChanged -> choose_target"<<std::endl;
          goto choose_target;
        }
        
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);

        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);

        Vector2f ballPositionRelative = theBallModel.estimate.position;
        //Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        theLookAtPointSkill(Vector3f(ballPositionRelative.x(), ballPositionRelative.y(), 0.f));
        
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theBallCarrierModel.staticApproachPoint);
      }
    }

    state(firstStepAlignment)
    {
      std::cout<<"firstStepRealignment"<<std::endl;
      transition
      {
        if(theBallModel.estimate.position.x() - theBallSpecification.radius < approachXRange.min
        && smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          goto secondStepAlignment;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

        float firstStepXPos = (approachXRange.max - approachXRange.min)/2;
        Pose2f firstStepPoseRelative = Pose2f(theLibCheck.angleToTarget(chosenTarget.x(), chosenTarget.y()), firstStepXPos, 0.f);
        theWalkToTargetSkill(Pose2f(1.f,1.f,1.f), firstStepPoseRelative);
      }
    }

    state(secondStepAlignment)
    {
      std::cout<<"secondStepRealignment"<<std::endl;
      transition
      {
        if(approachXRange.isInside(theBallModel.estimate.position.x())
        && approachYRange.isInside(theBallModel.estimate.position.y())
        && smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
        {
          goto kick;
        }

        //NEWLY ADDED
        if(state_time > realignmentTimeout)
        {
          goto kick;
        }

      }
      action
      {
        theActivitySkill(BehaviorStatus::realigning_to_ball);
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));

        Pose2f secondStepXPos = theLibCheck.glob2Rel(theBallCarrierModel.staticApproachPoint.translation.x(), theBallCarrierModel.staticApproachPoint.translation.y());
        Pose2f secondStepPoseRelative = Pose2f(theLibCheck.angleToTarget(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y()), secondStepXPos.translation.x(), secondStepXPos.translation.y());
        theWalkToTargetSkill(Pose2f(1.f,1.f,1.f), secondStepPoseRelative);
      }
    }

    state(kick)
    {
      //std::cout<<"kick"<<std::endl;
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)){
          goto searchForBall;
        }

        if(theLibCheck.distance(previousBallPosition, theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) > ballPositionChangedThreshold)
        {
          goto choose_target;
        }

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

//IMPORTANT: IDEA TO IGNORE ANGLE OF THE ROBOT WRT TARGET! 
//Rotate the local ball position wrt the target: local coordinates of the ball depend heavily on the angle

        //Refer to the drawing BallCarriern Alignment chart for case letters:
        //https://docs.google.com/drawings/d/1FcS2yrCbGkUmbWM1GRGHuXTYnEhcrbkovdwFayTGWkc/edit?usp=sharing

        //Case 0: if the robot is too far away from the ball, just walkToBall
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > approachXRange.max)
        {
          std::cout<<"kick: Case 0"<<std::endl;
          goto walkToBall;
        }
        else if(approachXRange.isInside(theBallModel.estimate.position.x()))
        {
          //Case B: Robot is in the X range but not in the Y range
          if(!approachYRange.isInside(theBallModel.estimate.position.y()))
          {
            std::cout<<"kick: Case B"<<std::endl;
            goto secondStepAlignment;
          }
          //else, it's ok to InWalkKick, as the robot is anyway well aligned
        }
        else if(theBallModel.estimate.position.x() >= approachXRange.max)
        {
          std::cout<<"kick: Case C"<<std::endl;
          //Case C: Robot is too far from the ball (and behind it, otherwise the theBallModel.estimate.position.x() would be negative)
          goto walkToBall;
        }
        else if(theBallModel.estimate.position.x() < approachXRange.min)
        {
          if(theBallModel.estimate.position.x() > theBallSpecification.radius)
          {
            //Case D: Robot is exactly behind the ball but too near 
            if(approachYRange.isInside(theBallModel.estimate.position.y()))
            {
              //Case D + wrong angle
              if(!smallBallAlignmentRange.isInside(calcAngleToTarget(chosenTarget).toDegrees()))
              {
                //If it is not aligned to the target first turn to the target, then the conditions will be checked again to perform further necessary alignments
                std::cout<<"kick: Case D + wrong angle"<<std::endl;
                goto turnToBall;
              }
              //else, it's ok to InWalkKick, as the robot is anyway well aligned
            }
            //Case E: the robot is not behind the ball
            else
            {
              std::cout<<"kick: Case E"<<std::endl;
              goto firstStepAlignment; //go back
            }
          }
          else
          {
            //Case F: Robot is between the target and the ball
            if(approachYRange.isInside(theBallModel.estimate.position.y()))
            {
              std::cout<<"kick: Case F"<<std::endl;
              goto walkToBall;
            }
            else
            {
              //Case G: the ball is to the side or behind wrt the robot and the robot is not in front of the ball
              std::cout<<"kick: Case G"<<std::endl;
              goto firstStepAlignment;
            }
          }
        }

        if(state_time > maxKickWaitTime)
        {  
          std::cout<<"KICK TIMEOUT"<<std::endl;
          goto choose_target;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::kicking_to_dynamic_target);
        
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);

        //Kick with the foot that is nearest to the ball
        if(theBallModel.estimate.position.y()>0)
        {
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
        else
        {
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theGoalTargetSkill(goalTarget);
        theSetTargetSkill(chosenTarget);
      }
    }

    state(searchForBall)
    {
      transition
      {
        std::cout<< "search" <<std::endl;
        if(theFieldBall.ballWasSeen())
          goto choose_target;
      }

      action
      {
        
        theActivitySkill(BehaviorStatus::searching_for_ball);

        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theLookForwardSkill();
        //TODO choose a direction and do the complete turn to find the ball
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

  int countNearbyObstacles(float distanceRadius) const
  {
    int obstacleCount = 0;
    for(auto obs : theObstacleModel.obstacles)
    {
      obstacleCount++;
    }  

    return obstacleCount;
  }
};

MAKE_CARD(ApproachAndCarryToGoalWithTwoStepRealignmentCard);
