/**
 * @file Representations/Modeling/HRIController.cpp
 * 
 * Representation holding all necessary data to model the execution state of the HRI routine
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "HRIController.h"


#ifndef ARC3D(id, xCenter, yCenter, zCenter, radius, fromAngle, angleSize, thickness, color)
#define ARC3D(id, xCenter, yCenter, zCenter, radius, fromAngle, angleSize, thickness, color) \
  do \
  { \
    constexpr Angle _angleStep = pi2 / 32.f; \
    Vector2f _from((xCenter) + std::cos(fromAngle) * (radius), (yCenter) + std::sin(fromAngle) * (radius)); \
    for(Angle _angle = _angleStep; _angle <= (angleSize) - _angleStep; _angle += _angleStep) \
    { \
      Vector2f _to((xCenter) + std::cos(_angle + (fromAngle)) * (radius), (yCenter) + std::sin(_angle + (fromAngle)) * (radius)); \
      LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
      _from = _to; \
    } \
    Vector2f _to((xCenter) + std::cos((fromAngle) + (angleSize)) * (radius), (yCenter) + std::sin((fromAngle) + (angleSize)) * (radius)); \
    LINE3D(id, _from.x(), _from.y(), (zCenter), _to.x(), _to.y(), (zCenter), thickness, color); \
  } \
  while(false)
#endif 

void HRIController::draw() const
{
  #ifdef TARGET_SIM
  // drawing of the model in the field view
  const HRIController& theHRIController = static_cast<const HRIController&>(Blackboard::getInstance()["HRIController"]);
  const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);


  switch(theHRIController.getCurrentActionType())
  {
    case HRI::ActionType::Idle:
    {
        DEBUG_DRAWING3D("representation:HRIController", "field")
        {
          const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);          
          ARC3D("representation:HRIController",
            theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(127,127,127,255)
          );
        };
        break;
    }
    case HRI::ActionType::ReachPosition:
    {
        DEBUG_DRAWING3D("representation:HRIController", "field")
        {
          const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
          const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);

          //SPHERE3D("representation:HRIController", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 10, ColorRGBA(0,255,0,255));  
          ARC3D("representation:HRIController",
                theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,255,0,255)
                );
          
          //SPHERE3D("representation:HRIController", theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10, 10, ColorRGBA(0,255,0,255));
          CROSS3D("representation:HRIController", theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10, 100, 10, ColorRGBA(255,0,0,255));
          
          CYLINDERARROW3D("representation:HRIController", 
            Vector3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 10),
            Vector3f(theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10),
            10, 50, 20, ColorRGBA(0,255,0,255));

        };
        break;
    }
    case HRI::ActionType::ReachBall:
    {
        DEBUG_DRAWING3D("representation:HRIController", "field")
        {
          const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
          const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);
          
          //SPHERE3D("representation:HRIController", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 10, ColorRGBA(0,255,0,255));
          ARC3D("representation:HRIController",
                theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,255,0,255)
                );
          
          //SPHERE3D("representation:HRIController", theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10, 10, ColorRGBA(0,255,0,255));
          ARC3D("representation:HRIController",
                theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,0,255,255)
                );
          
          CYLINDERARROW3D("representation:HRIController", 
            Vector3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 10),
            Vector3f(theHRIController.currentRobotDestination.x(), theHRIController.currentRobotDestination.y(), 10),
            10, 50, 20, ColorRGBA(0,255,0,255));
        };
        break;
    }
    case HRI::ActionType::CarryBall:
    {
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const BallCarrierModel& theBallCarrierModel = static_cast<const BallCarrierModel&>(Blackboard::getInstance()["BallCarrierModel"]);
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);

      Pose2f staticApproachPoint = theBallCarrierModel.staticApproachPoint();
      Pose2f dynamicApproachPoint = theBallCarrierModel.dynamicApproachPoint();

      Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
      float angleToBall = theLibCheck.angleToBall;
      DEBUG_DRAWING3D("representation:HRIController", "field")
      {

        //SPHERE3D("representation:HRIController", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 10, ColorRGBA(0,255,0,255));
        ARC3D("representation:HRIController",
              theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,255,0,255)
              );
        
        for(auto edge: theBallCarrierModel.obstacleAvoidancePlan)
        {
          SPHERE3D("representation:HRIController", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, edge.fromNode.radius,
            ColorRGBA(255,0,0,100));
          LINE3D("representation:HRIController", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, 
            edge.toNode.center.x(), edge.toNode.center.y(), 10, 
            5, ColorRGBA(255,0,0,255));
        }
        if(!theBallCarrierModel.isFallbackPath)
        {
          BallCarrierModel::Node prevStep = theBallCarrierModel.ballPath[0];
          SPHERE3D("representation:HRIController", 
            prevStep.center.x(), prevStep.center.y(), 10, 30,
            ColorRGBA(0,255,0,100));

          for(int i=1; i<theBallCarrierModel.ballPath.size(); i++)
          {
            BallCarrierModel::Node step = theBallCarrierModel.ballPath[i];
            SPHERE3D("representation:HRIController", 
              step.center.x(), step.center.y(), 10, 30,
              ColorRGBA(0,255,0,100));

            LINE3D("representation:HRIController", 
              prevStep.center.x(), prevStep.center.y(), 10, 
              step.center.x(), step.center.y(), 10, 
              5, ColorRGBA(0,255,0,255));
              prevStep = step;
          }
          //Draw the dynamic target (kick target chosen from the path)
        }
        else
        //If there is no path, use the fallback target
        {
          SPHERE3D("representation:HRIController", 
            globalBall.x(), globalBall.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          SPHERE3D("representation:HRIController", 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          LINE3D("representation:HRIController", 
            globalBall.x(), globalBall.y(), 10, 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            5, ColorRGBA(200,200,0,255));
        }
        float ARROW_LENGTH = 100;
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10),
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x()+ARROW_LENGTH*cos(theBallCarrierModel.dynamicTarget.rotation), theBallCarrierModel.dynamicTarget.translation.y()+ARROW_LENGTH*sin(theBallCarrierModel.dynamicTarget.rotation), 10),
              10, 50, 20, ColorRGBA(0,0,255,255));
        SPHERE3D("representation:HRIController", 
              theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
              30, ColorRGBA(0,0,255,255));
        
        //Draw the static approach point (farthest entry point to the approach area, determined by the chosen target)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10),
              Vector3f(staticApproachPoint.translation.x()+ARROW_LENGTH*cos(staticApproachPoint.rotation), staticApproachPoint.translation.y()+ARROW_LENGTH*sin(staticApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(255,0,0,255));
        SPHERE3D("representation:HRIController", 
              staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10, 30,
              ColorRGBA(255,0,0,255));
                      
        //Draw the dynamic approach point (entry point to the approach area, determined by the chosen target and by the distance of the robot from the ball)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10),
              Vector3f(dynamicApproachPoint.translation.x()+ARROW_LENGTH*cos(dynamicApproachPoint.rotation), dynamicApproachPoint.translation.y()+ARROW_LENGTH*sin(dynamicApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(0,255,0,255));
        SPHERE3D("representation:HRIController", 
              dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10, 30,
              ColorRGBA(0,0,255,255));
        
        //Draw the approach area
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.maximumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.minimumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
      };
      break;
    }
    case HRI::ActionType::Kick:
    {
  //TODO calcolare posizione di approccio come in BallCarrierModel
      DEBUG_DRAWING3D("representation:HRIController", "field")
      {
        float ARROW_LENGTH = 100;
        const HRIController& theHRIController = static_cast<const HRIController&>(Blackboard::getInstance()["HRIController"]);
        const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
        const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);
        const BallCarrierModel& theBallCarrierModel = static_cast<const BallCarrierModel&>(Blackboard::getInstance()["BallCarrierModel"]);

        Pose2f staticApproachPoint = theBallCarrierModel.staticApproachPoint();
        Pose2f dynamicApproachPoint = theBallCarrierModel.dynamicApproachPoint();

        //SPHERE3D("representation:HRIController", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 10, ColorRGBA(0,255,0,255));
        ARC3D("representation:HRIController",
              theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,255,0,255)
              );

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
        //Draw the static approach point (farthest entry point to the approach area, determined by the chosen target)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10),
              Vector3f(staticApproachPoint.translation.x()+ARROW_LENGTH*cos(staticApproachPoint.rotation), staticApproachPoint.translation.y()+ARROW_LENGTH*sin(staticApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(255,0,0,255));
        SPHERE3D("representation:HRIController", 
              staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10, 30,
              ColorRGBA(255,0,0,255));
                      
        //Draw the dynamic approach point (entry point to the approach area, determined by the chosen target and by the distance of the robot from the ball)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10),
              Vector3f(dynamicApproachPoint.translation.x()+ARROW_LENGTH*cos(dynamicApproachPoint.rotation), dynamicApproachPoint.translation.y()+ARROW_LENGTH*sin(dynamicApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(0,255,0,255));
        SPHERE3D("representation:HRIController", 
              dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10, 30,
              ColorRGBA(0,0,255,255));
        
        //Draw the approach area
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.maximumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.minimumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
        
        //SPHERE3D("representation:HRIController", globalBall.x(), globalBall.y(), 10, 10, ColorRGBA(255,0,0,255));
        //SPHERE3D("representation:HRIController", theHRIController.currentBallDestination.x(), theHRIController.currentBallDestination.y(), 10, 10, ColorRGBA(255,0,0,255));
        CYLINDERARROW3D("representation:HRIController", 
          Vector3f(globalBall.x(), globalBall.y(), 10),
          Vector3f(theHRIController.currentBallDestination.x(), theHRIController.currentBallDestination.y(), 10),
          10, 50, 20, ColorRGBA(0,255,0,255));
      };
      break;
    }
    case HRI::ActionType::CarryAndKickToGoal:
    {
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const BallCarrierModel& theBallCarrierModel = static_cast<const BallCarrierModel&>(Blackboard::getInstance()["BallCarrierModel"]);
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);

      Pose2f staticApproachPoint = theBallCarrierModel.staticApproachPoint();
      Pose2f dynamicApproachPoint = theBallCarrierModel.dynamicApproachPoint();

      Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
      float angleToBall = theLibCheck.angleToBall;
      DEBUG_DRAWING3D("representation:HRIController", "field")
      {

        //SPHERE3D("representation:HRIController", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 10, ColorRGBA(0,255,0,255));
        ARC3D("representation:HRIController",
              theRobotPose.translation.x(), theRobotPose.translation.y(), 10, 200, 0, pi2, 5, ColorRGBA(0,255,0,255)
              );
        
        for(auto edge: theBallCarrierModel.obstacleAvoidancePlan)
        {
          SPHERE3D("representation:HRIController", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, edge.fromNode.radius,
            ColorRGBA(255,0,0,100));
          LINE3D("representation:HRIController", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, 
            edge.toNode.center.x(), edge.toNode.center.y(), 10, 
            5, ColorRGBA(255,0,0,255));
        }
        if(!theBallCarrierModel.isFallbackPath)
        {
          BallCarrierModel::Node prevStep = theBallCarrierModel.ballPath[0];
          SPHERE3D("representation:HRIController", 
            prevStep.center.x(), prevStep.center.y(), 10, 30,
            ColorRGBA(0,255,0,100));

          for(int i=1; i<theBallCarrierModel.ballPath.size(); i++)
          {
            BallCarrierModel::Node step = theBallCarrierModel.ballPath[i];
            SPHERE3D("representation:HRIController", 
              step.center.x(), step.center.y(), 10, 30,
              ColorRGBA(0,255,0,100));

            LINE3D("representation:HRIController", 
              prevStep.center.x(), prevStep.center.y(), 10, 
              step.center.x(), step.center.y(), 10, 
              5, ColorRGBA(0,255,0,255));
              prevStep = step;
          }
          //Draw the dynamic target (kick target chosen from the path)
        }
        else
        //If there is no path, use the fallback target
        {
          SPHERE3D("representation:HRIController", 
            globalBall.x(), globalBall.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          SPHERE3D("representation:HRIController", 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          LINE3D("representation:HRIController", 
            globalBall.x(), globalBall.y(), 10, 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            5, ColorRGBA(200,200,0,255));
        }
        float ARROW_LENGTH = 100;
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10),
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x()+ARROW_LENGTH*cos(theBallCarrierModel.dynamicTarget.rotation), theBallCarrierModel.dynamicTarget.translation.y()+ARROW_LENGTH*sin(theBallCarrierModel.dynamicTarget.rotation), 10),
              10, 50, 20, ColorRGBA(0,0,255,255));
        SPHERE3D("representation:HRIController", 
              theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
              30, ColorRGBA(0,0,255,255));
        
        //Draw the static approach point (farthest entry point to the approach area, determined by the chosen target)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10),
              Vector3f(staticApproachPoint.translation.x()+ARROW_LENGTH*cos(staticApproachPoint.rotation), staticApproachPoint.translation.y()+ARROW_LENGTH*sin(staticApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(255,0,0,255));
        SPHERE3D("representation:HRIController", 
              staticApproachPoint.translation.x(), staticApproachPoint.translation.y(), 10, 30,
              ColorRGBA(255,0,0,255));
                      
        //Draw the dynamic approach point (entry point to the approach area, determined by the chosen target and by the distance of the robot from the ball)
        CYLINDERARROW3D("representation:HRIController", 
              Vector3f(dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10),
              Vector3f(dynamicApproachPoint.translation.x()+ARROW_LENGTH*cos(dynamicApproachPoint.rotation), dynamicApproachPoint.translation.y()+ARROW_LENGTH*sin(dynamicApproachPoint.rotation), 10),
              10, 50, 20, ColorRGBA(0,255,0,255));
        SPHERE3D("representation:HRIController", 
              dynamicApproachPoint.translation.x(), dynamicApproachPoint.translation.y(), 10, 30,
              ColorRGBA(0,0,255,255));
        
        //Draw the approach area
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.maximumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
        ARC3D("representation:HRIController",
              globalBall.x(), globalBall.y(), 10, theBallCarrierModel.minimumApproachDistance, 0, pi2, 5, ColorRGBA(0,0,0,255)
              );
      };
      break;
    }
    default:
    {
      DEBUG_DRAWING3D("representation:HRIController", "field")
      {};
    }    
  }
  #endif
}