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
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/spqr_representations/BallPath.h"
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
  REQUIRES(TeamBallModel),
  REQUIRES(RobotModel),
  REQUIRES(BallModel),

  REQUIRES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (float)(400.f) positionTh,
    (float)(300.f) smallPositionTh,
    
    (float)(0.f) ballOffsetX,
    (float)(0.f) ballOffsetY,

    (float)(0.f) theta,
    (float)(0.f) disty,
    (float)(0.f) distx,
    (float)(0.05) kp1,
    (float)(-0.008) kp2,
    (float)(0.008) kp3,
    (Vector3f)() error,


    (bool)(false) oppKickFlag,
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

     // < Copied from the Akshay's card StrikerContrastCard >
    common_transition {
        float xOffset = 140.0, yOffset = 0.0;
        Pose2f targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2;

        Pose2f footL = theLibCheck.rel2Glob(theRobotModel.soleLeft.translation.x(), theRobotModel.soleLeft.translation.y());
        Pose2f footOffL = theLibCheck.rel2Glob( theRobotModel.soleLeft.translation.x()+xOffset, theRobotModel.soleLeft.translation.y()+yOffset );

        Pose2f footR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x(), theRobotModel.soleRight.translation.y());
        Pose2f footOffR = theLibCheck.rel2Glob(theRobotModel.soleRight.translation.x()+xOffset, theRobotModel.soleRight.translation.y()-yOffset );

        targetPoint1.translation =  theTeamBallModel.position;
        
        if (theRobotPose.translation.y() < 0){
             targetPoint2.translation =  theTeamBallModel.position + Vector2f(0.0, 3000.0);    
        }else{
             targetPoint2.translation =  theTeamBallModel.position + Vector2f(0.0, -3000.0);    
        }
        if (theRobotPose.translation.y() < 0.0) {
            feedbackPoint1 = footR;
            feedbackPoint2 = footOffR;
        } else {
            feedbackPoint1 = footL;
            feedbackPoint2 = footOffL;
        }
        
        error = theLibCheck.getError(targetPoint1, targetPoint2, feedbackPoint1, feedbackPoint2);
        error.x() = error.x() - 40.0;
    }
    
    // < / Copied from the Akshay's card StrikerContrastCard >
    
    state(wallkToReceiverPosition)
    {
      transition
      {
        bool receiverArea = theLibCheck.C2ReceiverArea();

        float target_x, target_y;
        target_y = 1100.f;

        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        target_x = theLibCheck.C2EvaluateTarget(2).translation.x(); 
        target_y = theLibCheck.C2EvaluateTarget(2).translation.y();   

        //if (theRobotPose.translation.y() < 0) target_y = -target_y; 
        
        
        Pose2f relativeTarget = theLibCheck.glob2Rel(target_x, target_y);
        float relativeTargetX = relativeTarget.translation.x();        
        float relativeTargetY = relativeTarget.translation.y();        
        //std::cout<<"Relative target x:"<<relativeTargetX<<"\n";

        if(relativeTargetX < 0 and std::abs(relativeTargetY<700.f)){
            //std::cout<<"SKIPPO!\n";
            //goto turnAntiAround;
            goto goBackward;
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

        target_x = theLibCheck.C2EvaluateTarget(2).translation.x();    

        if (theRobotPose.translation.y() < 0) target_y = -target_y; 

        float x_diff = target_x - ball_x;
        //target_x = target_x + x_diff/2;

        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), theLibCheck.C2EvaluateTarget(2));
        
      }
    }

    state(turnAround){
        float angleTargetTreshold = 0.2;

        float target_x;
        float ball_x = theFieldBall.positionOnField.x();

        if (ball_x < 0) ball_x = -ball_x;

        if (ball_x < 2000.f) target_x = 1700.f;
        else target_x = 2300.f;

        Pose2f chosenTarget = theLibCheck.C2EvaluateTarget(2);

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
    
    /*state(turnAntiAround){
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
    }*/
    
     state(goBackward)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto start;
        
        // if (disty > 1.5 && oppKickFlag == true) {
        //   goto kickBall_ID2;
        // } 
        if (std::isnan(error.z())) {theta = 0.0;} else {theta = kp1*error.z();}    
        if (std::isnan(error.y())) {disty = 0.0;} else {disty = kp2*error.y();}
        if (std::isnan(error.x())) {distx = 0.0;} 
        else if (error.x() < 0.0 && error.x() > -500.0) {distx = kp3*2.0*error.x();} else {distx = kp3*error.x();}

        // if (std::isnan(error.x())) {distx = 0.0;} 
        // else if (error.x() < 0.0 && (error.z() > 90.0 || error.z() < -90.0)) {
        //   distx = -kp3*error.x();
        //   theta = 0.0;
        // } 
        // else if (error.x() < 0.0 && error.x() >= -500.0) {
        //   distx = -kp3*2.0*error.x();
        //   theta = 0.01*error.z();
        // } 
        // else {
        //   distx = kp3*error.x();
        // }

        //std::cout << "Error Norm: " << error << std::endl;
           
        float yReceiverArea = 1100.f;
        if(theRobotPose.translation.y()<0) yReceiverArea = -yReceiverArea;

        Pose2f actuallyChosenTarget = theLibCheck.C2EvaluateTarget(2);
        //Pose2f chosenTarget = Pose2f(actuallyChosenTarget.translation.x(),yReceiverArea);
        Pose2f chosenTarget = actuallyChosenTarget;
        float distanceToChosenTarget = (theRobotPose.translation - chosenTarget.translation).norm();

        if(distanceToChosenTarget>750.f) goto wallkToReceiverPosition;

        // if ((theBallModel.estimate.velocity).norm() == 0.0 && (theBallModel.estimate.position).norm() <=100.0)
        //   goto kickBall_ID2;

      }
      action
      {
        theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.f));
        
        float yReceiverArea = 1100.f;
        
        
        if(theRobotPose.translation.y()<0) yReceiverArea = -yReceiverArea;
        //std::cout<<"Y : "<<yReceiverArea<<"\n";
        Pose2f actuallyChosenTarget = theLibCheck.C2EvaluateTarget(2);
        //Pose2f chosenTarget = Pose2f(actuallyChosenTarget.translation.x(),yReceiverArea);
        Pose2f chosenTarget = actuallyChosenTarget;
        Pose2f relativeChosenTarget = theLibCheck.glob2Rel(chosenTarget.translation.x(), chosenTarget.translation.y());

        
        // theLookForwardSkill();
        // std::cout << error.z() << ", " << theta  <<  std::endl;
        // std::cout << error.y() << ", " << disty  <<  std::endl;
        // std::cout << theta << ", " << distx  << ", " << disty << std::endl;
        
        theWalkAtRelativeSpeedSkill(Pose2f(theta, relativeChosenTarget.translation.x()*0.1, relativeChosenTarget.translation.y()*0.1));
        // theWalkAtRelativeSpeedSkill(Pose2f(0.f, 1.f,0.f));
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
            
            Pose2f actuallyChosenTarget = theLibCheck.C2EvaluateTarget(2);
            //Pose2f chosenTarget = Pose2f(actuallyChosenTarget.translation.x(),yReceiverArea);
            Pose2f chosenTarget = actuallyChosenTarget;
            Pose2f relativeChosenTarget = theLibCheck.glob2Rel(chosenTarget.translation.x(), chosenTarget.translation.y());
            
            
            chosenTarget = theLibCheck.rel2Glob(-relativeChosenTarget.translation.x(), -relativeChosenTarget.translation.y());
            float distanceToChosenTarget = (theRobotPose.translation - chosenTarget.translation).norm();
            const Angle angleToTarget = calcAngleToTarget(chosenTarget); 
            if(relativeChosenTarget.translation.x() > 0){
                //std::cout<<"SKIPPO!\n";
                goto wallkToReceiverPosition;
            }
        
            if((not(std::abs(angleToTarget) < angleTargetTreshold)) and distanceToChosenTarget>300.f) goto goBackward; 
            if(distanceToChosenTarget>750.f) goto goBackward;
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



