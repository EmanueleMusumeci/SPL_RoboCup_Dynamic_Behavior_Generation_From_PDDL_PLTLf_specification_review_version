/**
 * @file Modules/BehaviorControl/BallCarrierModel/BallCarrierModelProvider.cpp
 *
 * This module was used for debugging purposes but all the values are now loaded directly 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModelProvider.h"

BallCarrierModelProvider::BallCarrierModelProvider()
{}

void BallCarrierModelProvider::update(BallCarrierModel& ballCarrierModel)
{
    ballCarrierModel.graphicalDebug = (GRAPHICAL_DEBUG==1 ? true : false);
    ballCarrierModel.obstacleAvoidanceArcAngleStep = pi2/OBSTACLE_AVOIDANCE_ARC_ANGLE_STEP_FACTOR;
    ballCarrierModel.obstacleAvoidancePlan.clear();
    ballCarrierModel.ballPath.clear();

    Vector2f goalTarget = theLibCheck.goalTarget(false);

    //Right now we're just pointing to the goal target without a specific angle
    Pose2f target = Pose2f(0.0, goalTarget.x(), goalTarget.y());
    //ballCarrierModel.dynamicTarget = Pose2f();

    //Set the goalTarget as a fallback target (if no path is found, a fake path between the ball and the goalTarget will be used)
    ballCarrierModel.dynamicTarget = target;
    ballCarrierModel.isFallbackPath = true;
    ballCarrierModel.isTargetOnGoal = true;
    ballCarrierModel.isTargetAPass = false;

    if(theGameInfo.state == STATE_PLAYING)
    {
        //Pose2f target = Pose2f(0.f, theFieldDimensions.xPosOpponentGroundline/2, 0.f);
        //Pose2f target = Pose2f(0.f, theFieldDimensions.xPosOpponentGroundline, 0.f);
        //TO DO Compute first a penalty area entry point to then determine the kicking angle to the goal target
        //This angle might be determined by the corridor planning
        
        Pose2f speed = Pose2f(0.8f,0.8f,0.8f);
        bool avoidPenaltyArea = false;
        
        std::vector<PathPlannerUtils::Node> plan = theLibPathPlanner.populatePlan(theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), target, speed, avoidPenaltyArea);
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
        //std::cout<<"ballCarrierModel.obstacleAvoidancePlan.size(): "<<ballCarrierModel.obstacleAvoidancePlan.size()<<std::endl;
        //for(auto edge : ballCarrierModel.obstacleAvoidancePlan)
        //{
        //    std::cout<<"Edge: from ("<<edge.fromNode.center.x()<<", "<<edge.fromNode.center.y()<<") to ("<<edge.toNode.center.x()<<", "<<edge.toNode.center.y()<<"), ";
        //}
        //std::cout<<std::endl;
        //std::cout<<"ballPath.size(): "<<ballPath.size()<<std::endl;
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
        
        //std::cout<<"ballCarrierModel.ballPath.size(): "<<ballCarrierModel.ballPath.size()<<std::endl;
        /*for(auto node : ballCarrierModel.ballPath)
        {
            std::cout<<"Node: ("<<node.center.x()<<", "<<node.center.y()<<"), ";
        }
        std::cout<<std::endl;*/
        

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
            //TODO choose a waypoint along the path
            //TODO determine an optimal offset from the chosen waypoint using APF
            //Set the dynamicTarget (TEMPORARILY USING THE NEXT PATH WAYPOINT)
            float angle = theLibCheck.angleToTarget(ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());
            ballCarrierModel.dynamicTarget = Pose2f(angle, ballCarrierModel.ballPath[1].center.x(), ballCarrierModel.ballPath[1].center.y());

            //If there are more than two steps in the path, the target is certainly not on the goal line
            ballCarrierModel.isTargetOnGoal = false;
        }
        
        //std::cout<<"ballCarrierModel.dynamicTarget: ("<<ballCarrierModel.dynamicTarget.translation.x()<<", "<<ballCarrierModel.dynamicTarget.translation.y()<<")"<<std::endl;
    }
}

MAKE_MODULE(BallCarrierModelProvider, behaviorControl)