/**
 * @file ApproachAndKickCard.cpp
 *
 * This file implements a behavior for approaching the ball and kick it to a given target.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Tools/Math/BHMath.h"
#include "Platform/SystemCall.h"
#include <string>
CARD(ApproachAndKickCard,
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
  CALLS(GoalTarget),

  REQUIRES(PassShare),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamPlayersModel),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(100.f) ballYThreshold,
    (float)(180.f) ballOffsetX,
    (float)(450.f) ballXnearTh,
    (Rangef)({170.f, 190.f}) ballOffsetXRange,
    (Rangef)({-350.f, 350.f}) approachYRange,
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

class ApproachAndKickCard : public ApproachAndKickCardBase
{

  // These two variables are used in order to let the robot say through PlaySound what is the distance from the target.
  double distanceConfirmed = 0.0;
  bool alreadyEnqueued = false;

  // Check whether the striker should kick,
  // which equates to "should not pass"
  // because if the dealer does not select this card
  // it checks ApproachAndPassCard next.
  bool preconditions() const override
  {
    //store a couple parameters so they're computed only once
    Pose2f goal_target = theLibCheck.goalTarget(false);

    //always choose to kick if one of the following conditions applies:
    if (
      //there is no pass available
      thePassShare.readyPass == 0 ||
      //there is a clear scoring opportunity (striker close enough to the goal with no opponents in sight)
      cleanShot(goal_target, theRobotPose, theTeamPlayersModel.obstacles)
    ) {
      return true;
    }
    //otherwise, the conditions branch depending on whether the striker
    //is under immediate pressure to send the ball *somewhere*.
    //This is true if the striker is close to the ball
    //and an opponent is immediately adjacent to the striker.
    //in these conditions, the striker can't afford to take too much time
    //walking around the ball.
    bool act_asap = (
      theFieldBall.positionRelative.squaredNorm()<sqr(300.f) &&
      sqrDistanceOfClosestOpponentToPoint(theRobotPose.translation)<sqr(500.0f)
    );
    if (act_asap) {
      //if the striker is pressed to act, then the choice between kick or pass
      //depends solely on which action is faster to execute in the immediate future
      //(remember that if the control flow gets here, there *is* an available pass).
      //Thus, choose to kick if it's easier than passing,
      //measured in terms of how much walk-around-the-ball the striker needs
      //(because we've already assume the striker is close to the ball)
      Angle kickAngle = Angle(atan2f(
        theRobotPose.translation.y()-goal_target.translation.y(),
        theRobotPose.translation.x()-goal_target.translation.x()
      ));
      Angle passAngle = Angle(atan2f(
        theRobotPose.translation.y()-thePassShare.passTarget.translation.y(),
        theRobotPose.translation.x()-thePassShare.passTarget.translation.x()
      ));
      Angle angDistToKick = theRobotPose.rotation.diffAbs(kickAngle);
      Angle angDistToPass = theRobotPose.rotation.diffAbs(passAngle);
      return angDistToKick <= angDistToPass;
    }
    else {
      //if the striker is not under immediate pressure,
      //it can actually reason and choose whether to kick or pass.
      // (TODO if we ever do a "walk with ball" card, it might be better to have these conditions in another card, lower in the hierarchy)
      //for now, the only discriminating condition in this case is:
      //refuse to pass if opponents are too close to target
      //(the pass routines only find the target w/ highest opp distance, but don't set a minimum threshold)
      //TODO tune this threshold
      return sqrDistanceOfClosestOpponentToPoint(thePassShare.passTarget.translation) < sqr(500.0f);
    }
  }

  bool postconditions() const override
  {
    //consider exiting if a pass is available, but because the striker deck is sticky
    //this is not a sufficient condition to pass: it can always return here if the preconditons still hold
    return thePassShare.readyPass == 1;
  }

  option
  {
    theActivitySkill(BehaviorStatus::approachAndKick);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
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

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall_far;

      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));

      }
    }

    state(walkToBall_far)
    {
      transition
      {

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionOnField.x() - ballOffsetX > theRobotPose.translation.x()){
          if(theFieldBall.positionOnField.x() < theRobotPose.translation.x() + ballXnearTh){
            if(approachYRange.isInside(theFieldBall.positionOnField.y() - theRobotPose.translation.y())){
              goto walkToBall_near;
            }
          }
        }
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldBall.positionOnField - Vector2f( ballOffsetX, 0.f)));

      }
    }

    state(walkToBall_near)
    {
      transition
      {

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(smallApproachXRange.isInside(theFieldBall.positionRelative.x())
            && smallApproachYRange.isInside(theFieldBall.positionRelative.y())){
                goto approach;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theWalkToTargetPathPlannerStraightSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldBall.positionOnField) - Pose2f(ballOffsetX, 50.f));
      }
    }

    state(approach)
    {
      transition{
        const Angle angleToTarget = calcAngleToTarget(theLibCheck.goalTarget(false));
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(theFieldBall.positionRelative.norm() < 0 ){
          goto turnToBall;
        }

        if(!smallApproachXRange.isInside(theFieldBall.positionRelative.x())){
          goto walkToBall_far;
        }


        if(std::abs(angleToTarget) < angle_target_treshold && ballOffsetXRange.isInside(theFieldBall.positionRelative.x())
            && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())){
                // We could let robot saying " KICK AT GOAL "
                goto kick;
        }
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        Vector2f target = theLibCheck.goalTarget(false);
        theGoalTargetSkill(target);
        theWalkToApproachSkill(target,
        ballOffsetX, ballOffsetY, true);

        double distanceTarget =  (target - theFieldBall.positionOnField).norm();
        // Since we are kicking, we don't want the ball to arrive just on the opponent goal line. So let's add 2 meters.
        distanceConfirmed = distanceTarget+2000.f;
        //const Angle angleToTarget = calcAngleToTarget(target);
        //std::cout<< "TAR_X:"<<target.x()<<"\tTAR_Y:"<<target.y()<<"\tDISTANCE TO TARGET:"<< distanceTarget<<"\tBallX:"<<theFieldBall.positionRelative.x()<<"\tBallY:"<<theFieldBall.positionRelative.y()<<"\tCHECKx:"<<ballOffsetXRange.isInside(theFieldBall.positionRelative.x())<<"\tCHECKy"<<ballOffsetYRange.isInside(theFieldBall.positionRelative.y())<<"\tyRange:["<<ballOffsetYRange.min<<","<<ballOffsetYRange.max<<"]\tangleToTarget:"<<std::abs(angleToTarget)<<"\tangleTreshold:"<<angle_target_treshold<<"\tNORM:"<<theFieldBall.positionRelative.norm()<<"\n";

      }
    }


    state(kick)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)){
          alreadyEnqueued = false;
          goto searchForBall;
        }

        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone())){
          alreadyEnqueued = false;
          goto start;
        }
      }

      action
      {
        if ( not alreadyEnqueued){
            alreadyEnqueued = true;
            std::string distanceTargetString = std::to_string(int(distanceConfirmed/1000.f));
            SystemCall::say("KICKING TO DISTANCE");
            SystemCall::say(distanceTargetString.c_str());
            SystemCall::say("METERS");

        }
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(Angle::fromDegrees(0.f), theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));

        theKickSkill(false, distanceConfirmed, false);
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

  // Adapting libCheck.canPass to check if the shooting line is clear of obstacles (of any type)
  // as a possible reason to prioritize shooting over passing
  bool cleanShot(Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>> obstacles) const
  {
    // relevant difference from canPass: striker must be close to opponent goal
    // because this test is pretty short-term
    if (shootingPose.translation.x() < theFieldDimensions.xPosOpponentPenaltyMark - 500.0f) {
      return false;
    }
    float m = 0;
    bool sameX = false;
    bool sameY = false;
    //se hanno la stessa x (più o meno)
    if(std::abs(shootingPose.translation.x() - targetPose.translation.x()) <= 0.5){
      sameX = true;
    }
    else{
      float m = shootingPose.translation.y() - targetPose.translation.y();
      if(std::abs(m) <= 0.5 ){
        sameY = true;
      }
      else{
        m /= shootingPose.translation.x() - targetPose.translation.x();
      }
    }
    float q = shootingPose.translation.y() - m * shootingPose.translation.x();
    float qThreshold = 100.f;
    //TODO inserire il vettore degli obstacles
    for(auto const& obstacle : obstacles){
      // relevant difference from canPass: only consider obstacles ahead of yourself
      if (obstacle.center.x() < shootingPose.translation.x()) {
        break;
      }
      if(sameX){
        if(std::abs(obstacle.center.x() - shootingPose.translation.x()) < qThreshold){
          std::cout<<"1"<<std::endl;
          return false;
        }
      }
      else if(sameY){
        if(std::abs(obstacle.center.y() - shootingPose.translation.y()) < qThreshold){
          std::cout<<"2"<<std::endl;
          return false;
        }
      }
      else{
        for(int i = -50; i < 50; i++){
          //se l'obstacle si trova su una retta del fascio
          if((obstacle.center.y() - m * obstacle.center.x() - q - (float) i * 10.f) <= 0.5f ){
            std::cout<<"3"<<std::endl;
            return false;
          }
        }
      }
    }
    return true;
  }

  //a simple auxiliary that gives the squared distance of the closest opponent to any given point
  float sqrDistanceOfClosestOpponentToPoint(Vector2f p) const {
    std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>> obstacles = theTeamPlayersModel.obstacles;
    //initializing the minimum distance to a value greater than the diagonal of the field
    //(therefore greater than any distance possible in the game)
    float minSqrDist = 2*theFieldDimensions.xPosOpponentFieldBorder + 2*theFieldDimensions.yPosLeftFieldBorder;
    for (Obstacle opp : obstacles) {
      if (opp.type == Obstacle::opponent) {
        float sqrdist = (p - opp.center).squaredNorm();
        if (sqrdist < minSqrDist) {
          minSqrDist = sqrdist;
        }
      }
    }
  }
};

MAKE_CARD(ApproachAndKickCard);
