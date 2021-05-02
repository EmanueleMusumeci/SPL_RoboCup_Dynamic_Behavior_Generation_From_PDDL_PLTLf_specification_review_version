/**
 * @file C1ApproachAndKickCard.cpp
 *
 * This file implements a behavior for passing the ball to a given target.
 * This is a slightly modified version of the ApproachAndKickCard.
 *d
 * @author Tommaso Carlini & Graziano Specchi
 */
 
// TODO: Scrivere il behaviour


#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Math/BHMath.h"

#include "Platform/SystemCall.h"
#include <string>

//TODO: Investigate and fix oscillation problem

CARD(C1ApproachAndKickCard,
{,
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),

  CALLS(WalkToApproach),

  CALLS(Stand),
  CALLS(Activity),

  REQUIRES(PassShare),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallCarrierModel),

  REQUIRES(GameInfo),
  REQUIRES(TeamPlayersModel),

  USES(BehaviorStatus),
  USES(TeamData),

  LOADS_PARAMETERS(
  {,
    (float) walkSpeed,
    (int) initialWaitTime,
    (int) ballNotSeenTimeout,
    (float) ballAlignThreshold_degrees,

    (float) ballYThreshold,
    (float) ballXnearTh,

    (Rangef) ballOffsetXRange,
    (Rangef) approachXRange,
    (Rangef) approachYRange,
    (Rangef) RangeX,
    (Rangef) RangeY,
    (Rangef) smallApproachYRange,
    (Rangef) smallApproachXRange,
    (Rangef) ballOffsetYRange,

    (float) ballOffsetX,
    (float) ballOffsetY,
    (int) minKickWaitTime,
    (int) maxKickWaitTime,
    (float) approachTreshold,
    (float) angle_target_treshold_degrees,

    (bool) DEBUG_MODE,
  }),
});


class C1ApproachAndKickCard : public C1ApproachAndKickCardBase
{
  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  Angle ballAlignThreshold = Angle::fromDegrees(ballAlignThreshold_degrees);
  Angle angle_target_treshold = Angle::fromDegrees(angle_target_treshold);

  double confirmedDistance = 0.f;

  bool alreadyQueued = false;  
  bool kickRequested = false;
  bool targetChosen = false;
  bool alreadyKickedOnce = false;

  Vector2f chosenTarget;
  Vector2f goalTarget;
  Vector2f goalTargetASAP;
  Vector2f previousBallPosition;

  // Because the Pass card was made lower-priority wrt the Kick card,
  // the dealer now only reaches this card if the Kick preconditions were not met.
  // Here we decide whether to pass (prioritized if possible)
  // or to carry the ball alone for the moment
  bool preconditions() const override
  {
      //AS OF CHALLENGE 1 RULES, the robot can shoot to goal only if the centre of the ball being within 1.3 meter
      //from the end field line
    if(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() > theFieldDimensions.xPosOpponentGroundline - 1300.0f) 
    {
      return true;
    }
    else
    {
        return false;
    }
  }

  //These conditions check when there is no good pass available anymore (or, indirectly, when the pass has been performed already)
  //or if the shootingTo target (where the ball is being kicked at) has been changed (for any reason).
  //NOTICE: the only code that sets thePassShare.readyPass to 0 is contained in PassShareProvider
  bool postconditions() const override
  {
//TODO go back to ball carrier if there is no way to kick
        return false;
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        std::cout<<"C1_APPROACH_AND_KICK: start"<<std::endl;
        if(state_time > initialWaitTime)
        {
            goto choose_target;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::approach_and_kick_start);
    
        theLookForwardSkill();
        theStandSkill();
      }
    }

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
            goto turnToBall;  
          }
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::choosing_target);
    
        std::cout<<"choose_target action"<<std::endl;
        goalTarget = theLibCheck.goalTarget(false);
        chosenTarget = goalTarget;
        previousBallPosition = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
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

    //Walk around, looking for the ball
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen()) //If the ball is seen...
        {
          goto turnToBall;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::searching_for_ball);

        theLookForwardSkill();
        //theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

    //Turn to the ball until acceptably aligned
    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
        {
          goto walkToBall_far;
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::aligning_to_ball);

        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    //Approach the ball if far
    state(walkToBall_far)
    {
      transition
      {
        Vector2f globalBallModel = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        if (approachXRange.isInside(globalBallModel.x() - theRobotPose.translation.x())) {
          if (approachYRange.isInside(globalBallModel.y() - theRobotPose.translation.y())) { 
              goto walkToBall_near;
          }
        }
      }

      action
      {
        theActivitySkill(BehaviorStatus::reaching_ball);

        Pose2f ballPose = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        //Pose2f targetPose = theLibCheck.C2EvaluateTarget(0);
        Pose2f targetPose = chosenTarget;

        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);

        Pose2f offset = theLibCheck.C2EvaluateApproach(targetPose);
        Pose2f target = Pose2f(angle, ballPose.translation + offset.translation);

        //std::cout << target.translation.x() << '\t' <<  target.translation.y() << '\n';
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.8f,0.8f,0.8f), target);
      }
    }

    //Approach the ball if near (only the transition conditions change wrt to walkToBall_far)
    state(walkToBall_near){
      transition
      {
        Vector2f globalBallModel = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }

        if (approachXRange.isInside((globalBallModel.x() - theRobotPose.translation.x())*1.3)){
          if (approachYRange.isInside((globalBallModel.y() - theRobotPose.translation.y())*1.3)) {
            goto approach;
            //std::cout << "ghello\n";
          }
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::aligning_to_ball);

        Pose2f ballPose = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        Pose2f targetPose = chosenTarget;
        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);
        Pose2f offset = theLibCheck.C2EvaluateApproach(targetPose);
        Pose2f target = Pose2f(angle, ballPose.translation + offset.translation);

        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.5f,0.5f,0.5f), target);
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approach){
      transition
      {
        //Pose2f point = theLibCheck.C2EvaluateTarget(0);
        Pose2f point = chosenTarget;
        point = theLibCheck.glob2Rel(point.translation.x(), point.translation.y());
        point = theLibCheck.rel2Glob(point.translation.x()-150.f, point.translation.y()+85.f);
        const Angle angleToTarget = calcAngleToTarget(point);
        float angle_threshold = .1f;
        //std::cout << "current Y:\t" << theFieldBall.positionRelative.y() << '\n';
        if (RangeX.isInside((theBallModel.estimate.position.x()))) {
          //std::cout << "y OK\t current X:\t" << theFieldBall.positionRelative.x() << '\n';
          if (RangeY.isInside((theBallModel.estimate.position.y()))) {
            //std::cout << "x OK\t current angle: \t" << angleToTarget << '\n';
            if (std::abs(angleToTarget) < angle_threshold) {
              std::cout << "Kicking\n";
              //goto wait;
              goto kick;
            }
          }
        }

      }

      action
      {
        theActivitySkill(BehaviorStatus::aligning_to_ball);

        Pose2f ballPose = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        Pose2f targetPose = chosenTarget;
        //float angle = theLibCheck.C2AngleToTarget_bis();
        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);        
        Pose2f ball = theBallModel.estimate.position;
        float x_ball = ball.translation.x();         
        float y_ball = ball.translation.y();         

        x_ball = x_ball - 150.f;
        y_ball = y_ball + 85.f;

        Pose2f globBall = theLibCheck.rel2Glob(x_ball, y_ball);
        Pose2f target = Pose2f(angle, globBall.translation.x(), globBall.translation.y());
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.8f, 0.8f, 0.8f), target);
      }
    }

    //Kick the ball to complete the passage
    state(kick)
    {
      transition
      {
        //if we're done kicking, go back to the initial state
        if(theMotionInfo.walkRequest.walkKickRequest.kickType != WalkKicks::none)
        {
          alreadyQueued = false;
          alreadyKickedOnce = true;
          goto start;
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::kicking_to_goal);

        if (!alreadyQueued){
            alreadyQueued = true;
            confirmedDistance = 9000.f;

            //std::cout << theTeamData.teammates.at(0).theRobotPose.translation.x() << '\t' << theTeamData.teammates.at(0).theRobotPose.translation.y() << '\n';
            std::string distanceTargetString = std::to_string(int(confirmedDistance/1000.f));
            //SystemCall::say("PASSING TO DISTANCE");
            //SystemCall::say(distanceTargetString.c_str());
            //SystemCall::say("METERS");

        }
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        theKickSkill(false, (float) confirmedDistance, false); // parameters: (kick_type, mirror, distance, armsFixed)

      }
    }
  }

  bool isAligned(Pose2f target_pose)
  {
    return calcAngleToTarget(target_pose) < ballAlignThreshold;
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

};

MAKE_CARD(C1ApproachAndKickCard);