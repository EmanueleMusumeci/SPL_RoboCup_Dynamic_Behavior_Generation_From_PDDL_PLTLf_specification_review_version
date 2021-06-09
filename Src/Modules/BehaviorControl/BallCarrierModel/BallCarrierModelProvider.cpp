/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.cpp
 *
 * This module contains all info necessary for the Ball Carrier model
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModelProvider.h"

BallCarrierModelProvider::BallCarrierModelProvider()
{}

void BallCarrierModelProvider::update(BallCarrierModel& ballCarrierModel)
{
    ballCarrierModel.graphicalDebug = (GRAPHICAL_DEBUG==1 ? true : false);

    ballCarrierModel.maximumApproachDistance = MAXIMUM_APPROACH_DISTANCE;
    ballCarrierModel.minimumApproachDistance = MINIMUM_APPROACH_DISTANCE;

    ballCarrierModel.staticApproachRadius = STATIC_APPROACH_RADIUS;

    ballCarrierModel.obstacleAvoidanceArcAngleStep = pi2/OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR;
    ballCarrierModel.obstacleAvoidancePlan.clear();
    ballCarrierModel.ballPath.clear();

    //Using heuristic-based goalTarget
    Vector2f goalTarget = theLibCheck.goalTarget(false, false);

    //Right now we're just pointing to the goal target without a specific angle
    //Pose2f target = Pose2f(0.0, goalTarget.x(), goalTarget.y());
    Pose2f target = Pose2f(0.0, theHRIController.currentBallDestination.x(), theHRIController.currentBallDestination.y());
    //ballCarrierModel.dynamicTarget = Pose2f();

    //Set the goalTarget as a fallback target (if no path is found, a fake path between the ball and the goalTarget will be used)
    ballCarrierModel.dynamicTarget = target;
    
    ballCarrierModel.isFallbackPath = true;
    ballCarrierModel.isTargetOnGoal = true;
    ballCarrierModel.isTargetAPass = false;

    //Default obstacle radius is 500 mm
    ballCarrierModel.dynamicGoalPostObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::goalpost);
    //ballCarrierModel.dynamicUprightRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::someRobot);

    //Find nearest obstacle to ball and its distance
    Vector2f nearestObstacleToBall;
    float nearestObstacleToBallDistance = -1;
    for(auto obs : theObstacleModel.obstacles)
    {
        float currentObsDistance = theLibCheck.distance(theBallModel.estimate.position, obs.center);
        if(nearestObstacleToBallDistance == -1 || theLibCheck.distance(theBallModel.estimate.position, obs.center) < nearestObstacleToBallDistance)
        {
            nearestObstacleToBall = Vector2f(obs.center.x(), obs.center.y());
            nearestObstacleToBallDistance = currentObsDistance;     
        }
    }

    ballCarrierModel.dynamicUprightRobotObstacleRadius = (nearestObstacleToBallDistance == -1 ? BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS : std::min(BASE_UPRIGHT_ROBOT_OBSTACLE_RADIUS, nearestObstacleToBallDistance));
    ballCarrierModel.dynamicReadyRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::someRobot);
    ballCarrierModel.dynamicFallenRobotObstacleRadius = theLibPathPlanner.getDefaultObstacleRadius(Obstacle::Type::fallenOpponent);
    ballCarrierModel.dynamicRadiusControlOffset = 100;

    if(theGameInfo.state == STATE_PLAYING)
    {
        
        Pose2f speed = Pose2f(0.8f,0.8f,0.8f);
        bool avoidPenaltyArea = false;
        std::vector<PathPlannerUtils::Node> plan = theLibPathPlanner.populatePlanWithCustomObstacleRadius(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), target, speed, avoidPenaltyArea,
                                                                                                            ballCarrierModel.dynamicGoalPostObstacleRadius,
                                                                                                            ballCarrierModel.dynamicUprightRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicReadyRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicFallenRobotObstacleRadius,
                                                                                                            ballCarrierModel.dynamicRadiusControlOffset);
        
        std::vector<PathPlannerUtils::Edge*> obstacleAvoidancePlan = theLibPathPlanner.createAvoidancePlan(plan);
        std::vector<Vector2f> ballPath = theLibPathPlanner.computePath(plan, ballCarrierModel.obstacleAvoidanceArcAngleStep);
        
        //NOTICE: Use goalTarget as a fallback target
        ballCarrierModel.dynamicTarget = target; 
        
        //Now we create new Node and Edge instances using the BallCarrierModel::Node and ::Edge definitions that are now STREAMABLE
        if(obstacleAvoidancePlan.size() > 0)
        {
            PathPlannerUtils::Edge* edge = obstacleAvoidancePlan[0];
            BallCarrierModel::Node fromNode(edge->fromNode->center, edge->fromNode->radius);
            BallCarrierModel::Node toNode;
            for(int i = 0; i<obstacleAvoidancePlan.size(); i++)
            {
                edge = obstacleAvoidancePlan[i];
                toNode = BallCarrierModel::Node(edge->toNode->center, edge->toNode->radius);
                ballCarrierModel.obstacleAvoidancePlan.push_back(BallCarrierModel::Edge(fromNode, toNode));
                fromNode = toNode;
            }
        }

        if(ballPath.size() > 0)
        {
            BallCarrierModel::Edge edge;
            BallCarrierModel::Node fromNode = BallCarrierModel::Node(ballPath[0], 0.f);
            ballCarrierModel.ballPath.push_back(fromNode);
            for(int i=1; i<ballPath.size(); i++)
            {
                Vector2f position = ballPath[i];
                BallCarrierModel::Node toNode(position, 0.f);
                edge = BallCarrierModel::Edge(fromNode, toNode);
                ballCarrierModel.ballPath.push_back(toNode);
            }
            ballCarrierModel.isFallbackPath = false;
        }
        else
        {
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation, 0.f));
            ballCarrierModel.ballPath.push_back(BallCarrierModel::Node(target.translation, 0.f));
            ballCarrierModel.isFallbackPath = true;
        }
        
        //Select the dynamic target (the next target the ball should reach along the path) (the default one is the fallback target that is the current goalTarget)
        if(ballPath.size()==2)
        {
            float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

            //If there are only two steps in the path (the ball position and the goalTarget), the target is on the goal line
            ballCarrierModel.isTargetOnGoal = true;
        }
        else
        {
            //Set the dynamicTarget
            float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

            //If there are more than two steps in the path, the target is certainly not on the goal line
            ballCarrierModel.isTargetOnGoal = false;
        }

        Vector2f ballPositionGlobal = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        float ballToTargetAngle = theLibCheck.angleBetweenPoints(ballPositionGlobal, ballCarrierModel.dynamicTarget.translation);
        
        float dynamicRadius;
        if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.maximumApproachDistance)
        {
            dynamicRadius = ballCarrierModel.maximumApproachDistance;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.minimumApproachDistance)
        {
            dynamicRadius = ballCarrierModel.minimumApproachDistance;
        }
        else if(theLibCheck.distance(theRobotPose, ballPositionGlobal) > ballCarrierModel.minimumApproachDistance 
        && theLibCheck.distance(theRobotPose, ballPositionGlobal) < ballCarrierModel.maximumApproachDistance) 
        {
            dynamicRadius = theLibCheck.distance(theRobotPose, ballPositionGlobal);
        }

        //Offsets depending on the distance of the robot from the ball
        float dynamicOffsetX = dynamicRadius * cos(pi + ballToTargetAngle);
        float dynamicOffsetY = dynamicRadius * sin(pi + ballToTargetAngle);
        
        //Maximum offsets
        float staticOffsetX = ballCarrierModel.staticApproachRadius * cos(pi + ballToTargetAngle);
        float staticOffsetY = ballCarrierModel.staticApproachRadius * sin(pi + ballToTargetAngle);

        ballCarrierModel.dynamicApproachPoint = Pose2f(-ballToTargetAngle, ballPositionGlobal.x() + dynamicOffsetX, ballPositionGlobal.y() - dynamicOffsetY);
        ballCarrierModel.staticApproachPoint = Pose2f(-ballToTargetAngle, ballPositionGlobal.x() + staticOffsetX, ballPositionGlobal.y() - staticOffsetY);
    }
}

MAKE_MODULE(BallCarrierModelProvider, behaviorControl)