/**
 * @file C2GoToReceiverPositionCard.cpp
 *
 * This file implements a behavior for reaching a receiver 
 * position, evaluated according to the ball position on the field
 *
 * @author Tommaso Carlini & Graziano Specchi
 */
 
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

    bool ownField = theLibCheck.C2OwnField();

    if (ownField) return false;
    else return true;
  }

  bool postconditions() const override
  {
    bool ownField = theLibCheck.C2OwnField();

    if (ownField) return true;
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        goto wallkToReceiverPosition;           
      }

      action
      {

      }
    }

    
    state(wallkToReceiverPosition)
    {
      transition
      {
        bool receiverArea = theLibCheck.C2ReceiverArea();

        float target_x, target_y;
        target_y = 1100.f;

        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        target_x = theLibCheck.C2EvaluateTarget().translation.x();    

        if (theRobotPose.translation.y() < 0) target_y = -target_y; 
        
        
        Pose2f relativeTarget = theLibCheck.glob2Rel(target_x, target_y);
        float relativeTargetX = relativeTarget.translation.x();        
        float relativeTargetY = relativeTarget.translation.y();        
        //std::cout<<"Relative target x:"<<relativeTargetX<<"\n";

        if(relativeTargetX < 0 and std::abs(relativeTargetY<700.f)){
            //std::cout<<"SKIPPO!\n";
            goto turnAntiAround;
        }
        if (receiverArea) goto turnAround;
      }

      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        float target_x, target_y;
        target_y = 1100.f;

        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        target_x = theLibCheck.C2EvaluateTarget().translation.x();    

        if (theRobotPose.translation.y() < 0) target_y = -target_y; 

        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(target_x, target_y));
        
      }
    }

    state(turnAround){
        float angleTargetTreshold = 0.2;

        float target_x;
        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        if (ball_x < 2000.f) target_x = 1700.f;
        else target_x = 2300.f;

        Pose2f chosenTarget = theLibCheck.C2EvaluateTarget();

        const Angle angleToTarget = calcAngleToTarget(chosenTarget); 
      transition
      {
        if(std::abs(angleToTarget) < angleTargetTreshold) goto waitForBall; 
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        float rotation_speed = .5f;
        if (angleToTarget < 0) rotation_speed = -rotation_speed;
        theWalkAtRelativeSpeedSkill(Pose2f(rotation_speed, 0.f,0.f));
      }
    }
    
    state(turnAntiAround){
        float angleTargetTreshold = 0.2;

        float target_x;
        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        if (ball_x < 2000.f) target_x = 1700.f;
        else target_x = 2300.f;

        float yReceiverArea = 1100.f;
        
        
        if(theRobotPose.translation.y()<0) yReceiverArea = -yReceiverArea;
        
        Pose2f actuallyChosenTarget = theLibCheck.C2EvaluateTarget();
        Pose2f chosenTarget = Pose2f(actuallyChosenTarget.translation.x(),yReceiverArea);
        
        Pose2f relativeChosenTarget = theLibCheck.glob2Rel(chosenTarget.translation.x(), chosenTarget.translation.y());
        
        
        chosenTarget = theLibCheck.rel2Glob(-relativeChosenTarget.translation.x(), -relativeChosenTarget.translation.y());
        
        const Angle angleToTarget = calcAngleToTarget(chosenTarget); 
        //const Angle oppositeAngleToTarget = -angleToTarget; 
        //std::cout<<"ANGLE \t"<<angleToTarget<<"\n";
        //std::cout<<"ANGLE AFTER MINOS \t"<<oppositeAngleToTarget<<"\n";
      transition
      {
        if(std::abs(angleToTarget) < angleTargetTreshold) goto walkBack; 
      }
      action{
        theLookAtPointSkill(Vector3f(theFieldBall.teamPositionRelative.x(), theFieldBall.teamPositionRelative.y(), 0.f));
        float rotation_speed = .5f;
        //if (angleToTarget < 0) rotation_speed = -rotation_speed;
        theWalkAtRelativeSpeedSkill(Pose2f(rotation_speed, 0.f,0.f));
      }
    }
    
    state(walkBack){
          transition{
            float angleTargetTreshold = 0.2;

            float target_x;
            float ball_x = theFieldBall.positionOnField.x();

            if (ball_x < 0) ball_x = -ball_x;

            if (ball_x < 2000.f) target_x = 1700.f;
            else target_x = 2300.f;

            float yReceiverArea = 1100.f;
            
            
            if(theRobotPose.translation.y()<0) yReceiverArea = -yReceiverArea;
            
            Pose2f actuallyChosenTarget = theLibCheck.C2EvaluateTarget();
            Pose2f chosenTarget = Pose2f(actuallyChosenTarget.translation.x(),yReceiverArea);
            
            Pose2f relativeChosenTarget = theLibCheck.glob2Rel(chosenTarget.translation.x(), chosenTarget.translation.y());
            
            
            chosenTarget = theLibCheck.rel2Glob(-relativeChosenTarget.translation.x(), -relativeChosenTarget.translation.y());
            float distanceToChosenTarget = (theRobotPose.translation - chosenTarget.translation).norm();
            const Angle angleToTarget = calcAngleToTarget(chosenTarget); 
            if(relativeChosenTarget.translation.x() > 0){
                //std::cout<<"SKIPPO!\n";
                goto wallkToReceiverPosition;
            }
        
            if((not(std::abs(angleToTarget) < angleTargetTreshold)) and distanceToChosenTarget>300.f) goto turnAntiAround; 
            if(distanceToChosenTarget>750.f) goto turnAntiAround;
        }
        action{
            theWalkAtRelativeSpeedSkill(Pose2f(0.f, -1.f,0.f));
        }
    
    
    }
    state(waitForBall){
      transition
      {
        float x_ball = theFieldBall.positionOnField.x();
        float x_nao = theRobotPose.translation.x();

        if (x_ball < 0) x_ball = -x_ball;
        if (x_nao < 0) x_nao = -x_nao;


        if (!theLibCheck.C2ReceiverArea()) goto start;
      }
      action{
        theStandSkill();
      }
    }

  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }
};

MAKE_CARD(C2GoToReceiverPositionCard);



