/**
 * @file C2ApproachAndPassCard.cpp
 *
 * This file implements a behavior for passing the ball to a given target.
 * This is a slightly modified version of the ApproachAndKickCard.
 *d
 * @author Tommaso Carlini & Graziano Specchi
 */
 
// TODO: Scrivere il behaviour


#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
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

CARD(C2ApproachAndPassCard,
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
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallCarrierModel),
  USES(BehaviorStatus),
  USES(TeamData),

  REQUIRES(GameInfo),

  REQUIRES(TeamPlayersModel),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) alignThreshold,
    (float)(100.f) ballYThreshold,
    (float)(180.f) ballOffsetX,
    (float)(450.f) ballXnearTh,
    (Rangef)({180.f, 180.f}) ballOffsetXRange,
    (Rangef)({-250.f, 250.f}) approachXRange,
    (Rangef)({-250.f, 250.f}) approachYRange,
    (Rangef)({100.f, 200.f}) RangeX,
    (Rangef)({-150.f, -75.f}) RangeY,
    (Rangef)({-150.f, 150.f}) smallApproachYRange,
    (Rangef)({150.f, 300.f}) smallApproachXRange,
    (float)(-75.f) ballOffsetY,
    (Rangef)({-85.f, -65.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (float)(1000.f) approachTreshold,
    (Angle)(20_deg) angle_target_treshold,
    (bool)(true) debugText,
  }),
});


class C2ApproachAndPassCard : public C2ApproachAndPassCardBase
{
  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  double distanceConfirmed = 0.0;
  bool alreadyEnqueued = false;

  // Because the Pass card was made lower-priority wrt the Kick card,
  // the dealer now only reaches this card if the Kick preconditions were not met.
  // Here we decide whether to pass (prioritized if possible)
  // or to carry the ball alone for the moment
  bool preconditions() const override
  {

    bool passingArea = theLibCheck.C2PassingArea();
    bool ownField = theLibCheck.C2OwnField();
    float distance = theLibCheck.distance(theRobotPose.translation, theFieldBall.positionOnField);

    //if (!passingArea || !ownField || distance > 500.f) return false;
    if (!ownField || !passingArea) return false;
    else return true;
  }

  //These conditions check when there is no good pass available anymore (or, indirectly, when the pass has been performed already)
  //or if the shootingTo target (where the ball is being kicked at) has been changed (for any reason).
  //NOTICE: the only code that sets thePassShare.readyPass to 0 is contained in PassShareProvider
  bool postconditions() const override
  {
    /*
    return thePassShare.readyPass == 0 ||
            std::abs(theBehaviorStatus.shootingTo.x())<std::abs(theRobotPose.translation.x());
    */
    if (
      thePassShare.readyPass == 0 ||
      std::abs(theBehaviorStatus.shootingTo.x())<std::abs(theRobotPose.translation.x())
    ) {
      //std::cout << "pass post: early check is true" << '\n';
      return true;
    }

    //give up passing if a shooting opportunity comes up,
    //because that has priority
    if (
      theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyMark - 1200.0f //&&
      //TO FRANCESCO FROM EMANUELE MUSUMECI: I had to relax a bit the precondition for the kick card 
      //theBallCarrierModel.isTargetOnGoal && !theBallCarrierModel.isFallbackPath
      //(theBallCarrierModel.isTargetOnGoal || theBallCarrierModel.isFallbackPath)
      
    ) {
      //std::cout << "Kicking opportunity arose, leaving pass card" << '\n';
      return true;
    }

    //otherwise check the same conditions as precond. to decide whether to keep passing
    bool shouldPass = theLibCheck.strikerPassCommonConditions(1);
    if (shouldPass) {
      //std::cout << "Still passing" << '\n';
      return true;
    }
    else {
      //std::cout << "Shouldn't pass anymore" << '\n';
      return false;
    }
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        //goto wait;
        if(state_time > initialWaitTime)
        {
          goto turnToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(wait)
    {
      transition
      {
      }

      action
      {

        Pose2f test = theLibCheck.C2EvaluateApproach(theLibCheck.C2EvaluateTarget(0));

        //Pose2f offset = theLibCheck.C2EvaluateApproach();
        //float angle = theLibCheck.C2AngleToTarget();

        //std::cout << "Offset " << offset.translation.x() << '\t' << offset.translation.y() << '\n';
        //std::cout << "angolo " << theLibCheck.radiansToDegree(angle) << '\n';
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
        if(std::abs(theFieldBall.positionRelative.angle()) < alignThreshold)
        {
          goto walkToBall_far;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    //Approach the ball if far
    state(walkToBall_far)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        if (approachYRange.isInside(theFieldBall.positionOnField.y() - theRobotPose.translation.y())) {
          if (approachXRange.isInside(theFieldBall.positionOnField.x() - theRobotPose.translation.x())) {} 
              goto walkToBall_near;
        }
      }

      action
      {
        Pose2f ballPose = Pose2f(theFieldBall.positionOnField);
        Pose2f targetPose = theLibCheck.C2EvaluateTarget(0);

        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);

        Pose2f offset = theLibCheck.C2EvaluateApproach(targetPose);
        Pose2f target = Pose2f(angle, theFieldBall.positionOnField + offset.translation);

        //std::cout << target.translation.x() << '\t' <<  target.translation.y() << '\n';
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.8f,0.8f,0.8f), target);
      }
    }

    //Approach the ball if near (only the transition conditions change wrt to walkToBall_far)
    state(walkToBall_near){
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }

        if (approachYRange.isInside((theFieldBall.positionOnField.y() - theRobotPose.translation.y())*1.3)){
          if (approachXRange.isInside((theFieldBall.positionOnField.x() - theRobotPose.translation.x())*1.3)) {
            goto approachToPass;
            //std::cout << "ghello\n";
          }
        }
      }
      action
      {
        Pose2f ballPose = Pose2f(theFieldBall.positionOnField);
        Pose2f targetPose = theLibCheck.C2EvaluateTarget(0);
        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);
        Pose2f offset = theLibCheck.C2EvaluateApproach(targetPose);
        Pose2f target = Pose2f(angle, theFieldBall.positionOnField + offset.translation);

        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(0.5f,0.5f,0.5f), target);
      }
    }

    //Approach the ball precisely (last robot movements before the kick)
    state(approachToPass){
      transition
      {
        Pose2f point = theLibCheck.C2EvaluateTarget(0);
        point = theLibCheck.glob2Rel(point.translation.x(), point.translation.y());
        point = theLibCheck.rel2Glob(point.translation.x()-150.f, point.translation.y()+85.f);
        const Angle angleToTarget = calcAngleToTarget(point);
        float angle_threshold = .1f;
        //std::cout << "current Y:\t" << theFieldBall.positionRelative.y() << '\n';
        if (RangeY.isInside((theFieldBall.positionRelative.y()))) {
          //std::cout << "y OK\t current X:\t" << theFieldBall.positionRelative.x() << '\n';
          if (RangeX.isInside((theFieldBall.positionRelative.x()))) {
            //std::cout << "x OK\t current angle: \t" << angleToTarget << '\n';
            if (std::abs(angleToTarget) < angle_threshold) {
              std::cout << "Passaggio\n";
              //goto wait;
              goto kick;
            }
          }
        }

      }

      action
      {
        Pose2f ballPose = Pose2f(theFieldBall.positionOnField);
        Pose2f targetPose = theLibCheck.C2EvaluateTarget(0);
        //float angle = theLibCheck.C2AngleToTarget_bis();
        float angle = theLibCheck.C2AngleBetween(targetPose, ballPose, false);        Pose2f ball = theFieldBall.positionRelative;
        float x_ball = ball.translation.x();         
        float y_ball = ball.translation.y();         

        x_ball = x_ball - 150.f;
        y_ball = y_ball + 85.f;

        Pose2f globBall = theLibCheck.rel2Glob(x_ball, y_ball);
        Pose2f target = Pose2f(angle, globBall.translation.x(), globBall.translation.y());
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
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
          alreadyEnqueued = false;
          goto start;
        }
      }
      action
      {
        if ( not alreadyEnqueued){
            alreadyEnqueued = true;
            distanceConfirmed = 2600.f;

            //std::cout << theTeamData.teammates.at(0).theRobotPose.translation.x() << '\t' << theTeamData.teammates.at(0).theRobotPose.translation.y() << '\n';
            std::string distanceTargetString = std::to_string(int(distanceConfirmed/1000.f));
            //SystemCall::say("PASSING TO DISTANCE");
            //SystemCall::say(distanceTargetString.c_str());
            //SystemCall::say("METERS");

        }
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theKickSkill(false, (float)distanceConfirmed, false); // parameters: (kyck_type, mirror, distance, armsFixed)

      }
    }
  }

  bool isAligned(Pose2f target_pose)
  {
    return calcAngleToTarget(target_pose) < alignThreshold;
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }


  bool shouldIPass() const
  {
    return true;
  }
};

MAKE_CARD(C2ApproachAndPassCard);
