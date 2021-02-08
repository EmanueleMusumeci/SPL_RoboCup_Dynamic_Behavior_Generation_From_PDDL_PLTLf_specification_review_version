/**
 * @file LibCheckProvider.cpp
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */
#include "Platform/Nao/SoundPlayer.h"
#include "LibCheckProvider.h"
#include <iostream>
#include <algorithm>
#define SQ(x) x*x

MAKE_MODULE(LibCheckProvider, behaviorControl);

int sign(float n){
  if(n > 0){
    return 1;
  }else if(n == 0){
    return 0;
  }else{
    return -1;
  }
}

LibCheckProvider::LibCheckProvider(): goalie_displacement(300.f),
        angleToGoal(0.f), angleToMyGoal(0.f), kickAngle(0.f), correctionKickAngle(0.f), ballOutOnLeft(false)
{
    SPQR::ConfigurationParameters();
}

void LibCheckProvider::update(LibCheck& libCheck)
{
  reset();
  libCheck.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen);
  libCheck.angleToGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  libCheck.isGoalieInStartingPosition = isGoalieInStartingPosition();
  libCheck.isBallInArea = isBallInArea();
  libCheck.isGoalieInAngle = isGoalieInAngle();
  libCheck.isGoalieInArea = isGoalieInArea();
  libCheck.isGoalieInKickAwayRange = isGoalieInKickAwayRange();
  libCheck.isBallInKickAwayRange = isBallInKickAwayRange();

  libCheck.norm = [&](float x, float y) -> float
  {
    return (float)(sqrt((x*x) + (y*y)));
  };

  libCheck.inc = [this](LibCheck::CheckedOutput outputToCheck) {inc(outputToCheck);};

  libCheck.setArm = [this](Arms::Arm arm)
  {
    setArmsInThisFrame[arm] = true;
  };
  
  libCheck.wasSetArm = [this](Arms::Arm arm) -> bool
  {
    return setArmsInThisFrame[arm];
  };
  
  libCheck.performCheck = [this](const MotionRequest& theMotionRequest)
  {
    checkOutputs(theActivationGraph, static_cast<LibCheck::CheckedOutput>(0), LibCheck::firstTeamCheckedOutput);
    checkMotionRequest(theActivationGraph, theMotionRequest);
  };
  
  libCheck.performTeamCheck = [this]()
  {
    checkOutputs(theTeamActivationGraph, LibCheck::firstTeamCheckedOutput, LibCheck::numOfCheckedOutputs);
  };
  
  libCheck.myReadyPosition = [this]() -> Pose2f{
    
    Pose2f strikerPose = Pose2f(0.f, -1000.f, 0.f);
    if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
      strikerPose = Pose2f(0.f, -500.f, 0.f);
    }else{
      strikerPose = Pose2f(0.f, -1000.f, 0.f);
    }
    
    Pose2f goaliePose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 200.f, 0.f);
    Pose2f defenderPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f);
    Pose2f jollyPose = Pose2f(0.f, -500.f, -1500.f);
    Pose2f supporterPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1700.f, 800.f);
    int activeTeam = 0;
    int lowerNumbers = 0;
    if(theRobotInfo.penalty == PENALTY_NONE){
      activeTeam++;
    }
    if(theRobotInfo.number == 1){
        return goaliePose;
    }
    for( auto teammate : theTeamData.teammates ){
      if(teammate.status != Teammate::PENALIZED && teammate.number != 1){
        activeTeam ++;
        if(teammate.number < theRobotInfo.number){
          lowerNumbers ++;
        }
      }
    }
    switch(activeTeam){
      case 0: return Pose2f(0.f,0.f,0.f); break;

      case 1: return strikerPose;
              break;

      case 2: if(lowerNumbers <= 0){
                return defenderPose;
              }else{
                return strikerPose;
              } 
              break;

      case 3: if(lowerNumbers == 2){
                  return strikerPose;
                }else if(lowerNumbers == 1){
                  return defenderPose;
                }else if(lowerNumbers == 0){
                  return supporterPose;
                }
                break;

      case 4: if(theRobotInfo.number == 3){
                return strikerPose;
              }else if(theRobotInfo.number == 2){
                return supporterPose;
              }else if (theRobotInfo.number == 5){
                return defenderPose;
              }else if(theRobotInfo.number == 4){
                return jollyPose;
              }
    }

    return strikerPose;
  };
libCheck.defenderDynamicDistance = [&]() -> float
    {
        float x1 = theRobotPose.translation.x();
        float y1 = theRobotPose.translation.y();
        float x3 = theTeamBallModel.position.x();
        float y3 = theTeamBallModel.position.y();
        float x2 = (float)theFieldDimensions.xPosOwnGroundline;   // first goalpost for defender
        float y2 = (y3/(std::abs(y3)+1))*750.f;  // first goalpost for defender
        float m = (y1-y2)/(x1-x2) ;
        float q = y1 - (((y1-y2)/(x1-x2))*x1) ;


        float distance = std::abs( y3 - (m*x3 +q) )/(std::sqrt( 1 + (m*m) ));

        return distance;
    };
     libCheck.nearestTemmate = [&]() -> Pose2f
    {
        Pose2f nearest = Pose2f(4500.f,0.f);
        for(const auto& mate : theTeamData.teammates){
                if( (mate.theRobotPose.translation- theRobotPose.translation).norm() < 500.f) {
                    nearest = mate.theRobotPose.translation;
                }
            }
        return nearest;
    };
  libCheck.distance = [this](const Pose2f p1, const Pose2f p2) -> float{
    return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) + 
      std::pow(p2.translation.y() - p1.translation.y(), 2) ) );
  };

  libCheck.refineTarget = [this](const Pose2f t, const float d) -> Pose2f{
    Rangef zeroTreshold = Rangef({-0.1f, 0.1f});
    float diffX = t.translation.x() - theRobotPose.translation.x();
    float diffY = t.translation.y() - theRobotPose.translation.y();

    //Check if target and robot have the same X
    if(zeroTreshold.isInside(diffX)){
      //if they also have the same y keep the same target
      if(zeroTreshold.isInside(diffY)){
        return t;
      }else{
        return Pose2f(theRobotPose.translation.x(), theRobotPose.translation.y() + (d * sign(diffY) ));
      }
    }// end of if X1 == X2

    //Check if target and robot have the same Y
    if(zeroTreshold.isInside(diffY)){
      //if they also have the same x keep the same target
      if(zeroTreshold.isInside(diffX)){
        return t;
      }else{
        return Pose2f( theRobotPose.translation.x() + (d * sign(diffX)) , theRobotPose.translation.y());
      }
    }// end of if Y1 == Y2

    //if the distance is still lower than d, than just return t
    if(distance(theRobotPose, t) <= d ){
      return t;
    }

    // Y = mX + q
    float m = diffY/diffX;
    float q = -theRobotPose.translation.y() -theRobotPose.translation.x()*m;  
    // just for readability
    float x1 = theRobotPose.translation.x();
    float y1 = theRobotPose.translation.y();
    
    float a = SQ(m) + 1;
    float b = 2*m -2*y1*m -2*x1;
    float c = SQ(q) -2*y1*q +SQ(y1) +SQ(x1) -SQ(d);
    
    //Use the delta formula
    float X = (-b + std::sqrt(SQ(b) - 4*a*c))/(2*a);
    //classical line equation
    float Y = m*X + q;

    return Pose2f(X,Y);
  };

  /*
  @author Emanuele Musumeci
  Project my line of sight (the direction I'm looking at) on the opponent groundline
  */
  libCheck.projectGazeOntoOpponentGroundline = [this]() -> float{
      const float EPS = 1e-6f;
      float rotation = theRobotPose.rotation;
      //To avoid tan singularities
      if(std::abs(rotation) == pi/2) rotation -= EPS;
      return theRobotPose.translation.y() + std::abs(theFieldDimensions.xPosOpponentGoal - theRobotPose.translation.x())*tan(theRobotPose.rotation);
  };

  /*
  @author Emanuele Musumeci
  Project a point on the opponent groundline
  */
  libCheck.projectPointOntoOpponentGroundline = [this](float x, float y) -> float{
    return ((theFieldDimensions.xPosOpponentGroundline - theRobotPose.translation.x())/(x - theRobotPose.translation.x()))*(y - theRobotPose.translation.y()) + theRobotPose.translation.y();
  };

  /*
  @author Emanuele Musumeci
  Determine the distance between the projection of the obstacle obs and of my line of sight 
  (the direction I'm looking at) on the opponent groundline
  */
  libCheck.gazeToObstacleProjectionDistanceOntoOpponentGroundLine = [this](Obstacle obs) -> float {
    float obs_center_proj_dist = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
    float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.right.y());
    float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());
    float my_gaze = projectGazeOntoOpponentGroundline();
    if(my_gaze<obs_right_proj)
    {
      return abs(obs_right_proj-my_gaze);
    }
    else if(my_gaze>obs_right_proj && my_gaze<obs_left_proj)
    {
      return 0.f;
    }
    else
    {
      return abs(my_gaze-obs_left_proj);
    } 
  };

  /*
  @author Graziano Specchi
  Provide a utility value for a certain targetable segment on the opponent goal line
  */
libCheck.areaValueHeuristic = [this](const float leftLimit, const float rightLimit)  -> float 
{
  // Useful notes for the reader:
  //       leftLimit is the left limit of THIS free area, rightLimit is the right one. 
  //       Note that the axis is directed to the left, then leftLimit > righLimit.

    std::vector<float> opponents_distances;
    std::vector<float> teammates_distances;
    
    Pose2f freeAreaPoseleftLimit= Pose2f(theFieldDimensions.xPosOpponentGroundline,leftLimit);
    Pose2f freeAreaPoserightLimit= Pose2f(theFieldDimensions.xPosOpponentGroundline,rightLimit);

    for(auto obs : theTeamPlayersModel.obstacles){
        
        if ((obs.center.x()==theFieldDimensions.xPosOpponentGoalPost) && (obs.center.y()==theFieldDimensions.yPosLeftGoal || obs.center.y()==theFieldDimensions.yPosRightGoal)){
          continue;  // don't consider the poles
        }
        if ((obs.center.x()==-theFieldDimensions.xPosOpponentGoalPost) && (obs.center.y()==theFieldDimensions.yPosLeftGoal || obs.center.y()==theFieldDimensions.yPosRightGoal)){
          continue;  // don't consider the poles
        }
        if(theRobotPose.translation.x()>obs.center.x()){
          continue; // if the obstacle is behind me, don't consider me
        }

        float final_distance=0;

        //###########     Considering projection wrt my eyes 

   
        float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
        float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
        float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());


        // IT'S NOT POSSIBLE THAT THIS PROJECTION IS INTO SOME FREE_AREAS. Then, just check if it is left or right wrt the obstacle.

        if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
          final_distance=obs_right_proj-leftLimit;

        }

        if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
          final_distance=rightLimit-obs_left_proj;
        }

        // Differentiate the obstacles in two sets: one of oppontents, one of teammates.
        if(obs.isOpponent()){
          opponents_distances.push_back(final_distance);
        }
        if(obs.isTeammate()){
          teammates_distances.push_back(final_distance);
        }      
       

    }

    // Although they are in two different vectors, they are treated in the same way. 
    // However if one in the future wants to use this feature, it is ready to be exploited.
    // Let's consider just the closest opponent
    float minimo_opponents=0;
    bool firstTime=true;
    for(auto distance : opponents_distances){
      if(firstTime){
        firstTime=false;
        minimo_opponents=distance;
      }else if(minimo_opponents>distance){
        minimo_opponents=distance;
      }
    }

    //Let's use the same minimum variable for both opponents and teammetes ( no distinction )
    float minimo_teammates=0;
    firstTime=false;
    for(auto distance : teammates_distances){
      if(firstTime){
        firstTime=false;
        minimo_opponents=distance;
      }
      if(minimo_opponents>distance){
        minimo_opponents=distance;
      }
    }
    // final utility is always positive.
    float final_utility=100000+minimo_opponents+minimo_teammates;

    return final_utility;

    //Ideas that can be exploited in order to improve this function:
    /*
    1) First criteria: vicinity to jolly (or in alternative the second robot nearest to the ball, as the first one is the striker)
    2) Second criteria: number of opponent in intercept range
    3) Third criteria: vicinity of enemy goalie (is this possible?)
    4) Width of parent area
    5) Penalize the poles, but this can be done at an higher level of abstraction
    */
  };
  //#####################

  libCheck.areOverlappingSegmentsOnYAxis = [this](float l1, float r1, float l2, float r2) -> bool {
    return ((l1<l2 && l1>r2) || (r1<l2 && r1>r2) || (l2<l1 && l2>r1) || (r2<l1 && r2>r1));
  };

  /*
  @author Emanuele Musumeci
  @author Graziano Specchi
  Return a vector of FreeGoalTargetableAreas (structures that represent a segment free from obstacles, 
  hence targetable, on the opponent goal line)
  */
  libCheck.computeFreeAreas = [this](float minimumDiscretizedAreaSize) -> std::vector<FreeGoalTargetableArea>
  {
    /*
    NOTICE:
    begin = leftLimit
    end = rightLimit
    */
  
    Pose2f myPose = Pose2f(theRobotPose.translation);

    float GOAL_TARGET_OBSTACLE_INFLATION = theOpponentGoalModel.goalTargetObstacleInflation;
    float GOAL_TARGET_AREA_DISCRETIZATION = theOpponentGoalModel.useAreaDiscretization;

    //vector of opponents
    std::vector<Obstacle> opponents;
    //vector of poles
    std::vector<Obstacle> poles;

    for(auto obs : theTeamPlayersModel.obstacles){
      /*NOTICE: poles are added statically (a priori) by the vision system
        so the 4 goal poles will always be in the obstacle list*/
      switch(obs.type)
      {
        case Obstacle::Type::goalpost:
        {
          if(obs.type==Obstacle::Type::goalpost && obs.center.x()>0)
          {
            //std::cout<<"Found opponent goal post: (left:"<<obs.left.y()<<", right:"<<obs.right.y()<<")\n";
            poles.push_back(obs);
          }
          break;
        }
        default:
        {
          if(obs.center.x()>theRobotPose.translation.x() || obs.left.x()>theRobotPose.translation.x() || obs.right.x()>theRobotPose.translation.x())
          {
            opponents.push_back(obs);
          }
          break;
        }
      }
    }

    Pose2f leftPole, rightPole;

    if(poles.size()==0)
    {
      leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),730.);
      if(leftPole.translation.y()>730) leftPole.translation.y()=730; 
      rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),-730.);
      if(rightPole.translation.y()<-730) leftPole.translation.y()=-730; 
    }
    else
    {
      
      leftPole.translation.x() = poles.at(0).right.x();
      if(leftPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundline) leftPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundline; 

      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),poles.at(0).right.y());
      if(leftPole.translation.y()>730) leftPole.translation.y()=730; 

      rightPole.translation.x() = poles.at(1).left.x();
      if(rightPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundline) rightPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundline; 

      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),poles.at(1).left.y());
      if(rightPole.translation.y()<-730) rightPole.translation.y()=-730; 
      
      /*
      WORK IN PROGRESS - Trying to use the projection of the poles on the groundline as left/right limit of the
      goal line (useful in case the player is particularly angled wrt the opponent goal)
      
      Vector2f leftPoleRightLimit(
        1/tan(theRobotPose.rotation+asin(POLE_RADIUS/distance(theRobotPose,poles.at(0).center))),
        poles.at(0).center.y() - tan(theRobotPose.rotation+asin(POLE_RADIUS/distance(theRobotPose,poles.at(0).center)))
      );
      leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),leftPoleRightLimit.y());
      
      Vector2f rightPoleLeftLimit(
        1/tan(theRobotPose.rotation-asin(POLE_RADIUS/distance(theRobotPose,poles.at(1).center))),
        poles.at(1).center.y() - tan(theRobotPose.rotation-asin(POLE_RADIUS/distance(theRobotPose,poles.at(1).center)))
      );
      rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),rightPoleLeftLimit.y());
      */

      //std::cout<<"Left goal pole projection: ("<<leftPole.translation.x()<<","<<leftPole.translation.y()<<")\n";
      //std::cout<<"Right goal pole projection:"<<rightPole.translation.x()<<","<<rightPole.translation.y()<<")\n";
    }
    
    //FREE AREAS COMPUTATION

    std::vector<float> leftPoints;
    std::vector<float> rightPoints;
    std::vector<FreeGoalTargetableArea> freeAreas;

    
    Obstacle swapper;

    /*1) Sort the opponents in vector based on the y coordinate of their left points 
    (for each obstacle, the leftmost point I see)*/
    for(int i = 0; i < opponents.size(); i++){
        for(int k = 0; k < opponents.size(); k++){
            float firstLeft = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y());
            float secondLeft = projectPointOntoOpponentGroundline(opponents.at(k).left.x(),opponents.at(k).left.y());
            if(firstLeft > secondLeft ){
                swapper = opponents.at(k);
                opponents.at(k) = opponents.at(i);
                opponents.at(i) = swapper;
            }
        }
    }
    /*std::cout<<"Opponents: [";
    for(int i=0;i<opponents.size();i++)
    {
      std::cout<<"("<<std::to_string(opponents.at(i).center.x())+","<<opponents.at(i).center.y()<<"), ";
    }
    std::cout<<"]\n";*/

    /*2) Find overlapping obstacles and merge them into a single one. For each obstacle (including the
        ones obtained by merging) save the projections on the goal line of its left and right point, in order
        to populate a list of obstacles from the player's point of view.

        NOTICE: the obstacles are ordered from left to right

        NOTICE: GOAL_TARGET_OBSTACLE_INFLATION is used as an "obstacle inflation" factor
        to enlarge obstacles (as the visualization system makes the opponent robots smaller than their
        feet width
    */
    
    if(opponents.size()>0)
    {
      float leftPointY = projectPointOntoOpponentGroundline(opponents.at(0).left.x(),opponents.at(0).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(0).left);
      float rightPointY = projectPointOntoOpponentGroundline(opponents.at(0).right.x(),opponents.at(0).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(0).right);
      if(opponents.size()==1)
      {
        //If the obstacle projection is at least partially inside the goal line add it
        if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
        {
          //std::cout<<"1 Adding single obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
          leftPoints.push_back(leftPointY);
          rightPoints.push_back(rightPointY);
        }
      }
      else if(opponents.size()>1)
      {
        int i=1;
        bool wereOverlapping;

        /*
          For each obstacle in the list, compare it against the next one to check for overlapping and,
          depending on the kind of overlapping, merge them accordingly or add them separately
        */
        while(i<opponents.size()+1)
        {
          float nextLeftPointY, nextRightPointY;
          if(i==opponents.size())
          {
            /*
              If leftPoint and rightPoint identify the last obstacle in the opponents list, use 
              the right pole as the next obstacle to compare for overlapping
            */
            nextLeftPointY = projectPointOntoOpponentGroundline(poles.at(1).left.x(),poles.at(1).left.y());
            nextRightPointY = projectPointOntoOpponentGroundline(poles.at(1).right.x(),poles.at(1).right.y());
          }
          else
          {
            /*
              Else use the next obstacle in the list
            */
            nextLeftPointY = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(i).left);
            nextRightPointY = projectPointOntoOpponentGroundline(opponents.at(i).right.x(),opponents.at(i).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(i).right);
          }
          //std::cout<<"Iteration #"<<i<<": LeftPointY: "+std::to_string(leftPointY)+", RightPointY: "+std::to_string(rightPointY)+"\n";
          //std::cout<<"Iteration #"<<i<<": nextLeftPointY: "+std::to_string(nextLeftPointY)+", nextRightPointY: "+std::to_string(nextRightPointY)+"\n";
          
          /*
            Check for overlapping: there are three cases to manage
            1) One obstacle is inside the other: do nothing
            2) Obstacles overlap only partially: merge the obstacles
            3) No overlap: add the first obstacle and pass to the second one
          */
          if(areOverlappingSegmentsOnYAxis(leftPointY, rightPointY, nextLeftPointY, nextRightPointY))
          {
            //std::cout<<"Overlapping: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+") with ("+std::to_string(nextLeftPointY)+","+std::to_string(nextRightPointY)+")\n";
            
            if(leftPointY>nextLeftPointY && rightPointY<nextRightPointY)
            {
              //1) One obstacle is inside the other: do nothing

              //std::cout<<"CASE 1\n";
              //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
            }
            else
            {
              //2) Obstacles overlap only partially: merge the obstacles

              //std::cout<<"CASE 2\n";
              rightPointY = nextRightPointY;
              
              //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  

            }
            wereOverlapping=true;
          }
          else
          {  
            //3) No overlap: add the first obstacle and pass to the second one
            if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
            {
              //Add the obstacle projection only if it falls (even only partially) inside the goal line
              //std::cout<<"2 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
              leftPoints.push_back(leftPointY);
              rightPoints.push_back(rightPointY);
            }
            leftPointY = nextLeftPointY;
            rightPointY = nextRightPointY;
            wereOverlapping=false;
          }

          i++;
        }

        //Add last remaining obstacle points (only in case where it overlaps right pole)
        if(wereOverlapping)
        {  
          if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
          {
            //Add the obstacle projection only if it falls (even only partially) inside the goal line
            //std::cout<<"3 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
            leftPoints.push_back(leftPointY);
            rightPoints.push_back(rightPointY);
          }
        }      
      }
    }

    //std::cout<<"End of overlap check\n\n";
    
    /*3) Now that we have left and right points of the obstacles
      3.1) We first determine if there is any free area (or if a single obstacle 
           projection overlaps the whole goal line)
      3.2) We shrink the left and right limit (begin/end) of the goal line if there are
           obstacles overlapping the left/right poles
    */
    //determines if all obstacles are outside goal
    bool noneInside = true;

    //Consider my angle in inflating the pole
    float begin = leftPole.translation.y();
    float end = rightPole.translation.y();

    for(int i = 0; i < leftPoints.size(); i++){

        //3.1)
        //at least one point inside goal
        if(leftPoints.at(i)<leftPole.translation.y() && rightPoints.at(i)>rightPole.translation.y() || 
        leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i)<leftPole.translation.y() || 
        leftPoints.at(i)>rightPole.translation.y() && rightPoints.at(i)<rightPole.translation.y())
        {
          noneInside = false;
        }
//WATCH OUT, TEMPORARY SOLUTION: If an obstacle projection cover the whole goal area I return an empty area list
        if(leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i) < rightPole.translation.y())
        {
          return freeAreas;
        }

        //3.2)
        //left obstacle border outside goal, right border inside
        if(leftPoints.at(i) > leftPole.translation.y() && rightPoints.at(i) < leftPole.translation.y()){
            begin = rightPoints.at(i);
        }
        //right obstacle border outside goal, left border inside
        if(leftPoints.at(i) > rightPole.translation.y() && rightPoints.at(i) < rightPole.translation.y()){
            end = leftPoints.at(i);
        }
    }

    /*4) Build the free targetable segments vector
      4.1) If there is no targetable segment, return the empty vector
      4.2) Else populate the freeAreas vector
    */

    //4.1)
    if(noneInside == true){
      freeAreas.push_back(FreeGoalTargetableArea(begin, end, 1));
      //std::cout<<"NONE INSIDE\n";
        return freeAreas;
    }
    std::vector<float> freeAreasPoints;
    freeAreasPoints.push_back(begin);

    //4.2)
    for(int i = 0; i < leftPoints.size(); i++){
        if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
            freeAreasPoints.push_back(leftPoints.at(i));
        }
        if(rightPoints.at(i) < begin && rightPoints.at(i) > end ){
            freeAreasPoints.push_back(rightPoints.at(i));
        }
    }
    freeAreasPoints.push_back(end);

    sort(freeAreasPoints.begin(),freeAreasPoints.end());

    /*std::cout<<"FreeAreasPoints: [";
    for(int i=0;i<freeAreasPoints.size();i++)
    {
      std::cout<<std::to_string(freeAreasPoints.at(i))+",";
    }
    std::cout<<"]\n";*/

    /*5A) Apply discretization to the targetable segments, by dividing targetable areas in smaller segments,
        in order to assign them a utility value
    */

    if(GOAL_TARGET_AREA_DISCRETIZATION)
    {
      for(int i = freeAreasPoints.size()-1; i-1>=0; i-=2)
      {
        float beginning = freeAreasPoints.at(i);
        float end = freeAreasPoints.at(i-1);
        float size = std::abs(end-beginning);
        if(size>minimumDiscretizedAreaSize)
        {
          int numberOfDiscretizedAreasInFreeArea = floor(size/minimumDiscretizedAreaSize);

          float discretizedAreaSize = size/numberOfDiscretizedAreasInFreeArea;

          float firstPoint = beginning;
          for(int i = numberOfDiscretizedAreasInFreeArea -1; i>0; i--)
          {
            float lastPoint = firstPoint-discretizedAreaSize;
            freeAreas.push_back(FreeGoalTargetableArea(firstPoint,lastPoint,areaValueHeuristic(firstPoint,lastPoint)));
            firstPoint = lastPoint;
          }
            freeAreas.push_back(FreeGoalTargetableArea(firstPoint,end,areaValueHeuristic(firstPoint,end)));
        }
        else
        {
          continue;
        }
        
      }

      /*
        6) Sort the result of the discretization on its utility value
      */

      //Sort the freeAreas vector in a decreasing order of utility values
      sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
      /*for(const auto& free: freeAreas)
      {
        std::cout<<free.value<<" ";
      }
      std::cout<<"\n";*/
    }
    else
    {
      /*
        5B) Do not discretize free areas: the whole area between opponent projections is used:
            might be imprecise and less effective and also will require longer times to align with the ball
      */
      for(int i = freeAreasPoints.size() -1; i-1 >= 0; i-=2)
      {
        freeAreas.push_back(FreeGoalTargetableArea(freeAreasPoints.at(i), freeAreasPoints.at(i-1), areaValueHeuristic(freeAreasPoints.at(i),freeAreasPoints.at(i-1))));
      }
      sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
    }
    
    /*std::cout<<"Discretized FreeAreas: [";
    for(int i=0; i<freeAreas.size(); i++)
    {
      std::cout<<"("<<std::to_string(freeAreas.at(i).end)<<","<<std::to_string(freeAreas.at(i).begin)<<"),";
    }
    std::cout<<"]\n";*/

    //std::cout<<"Role: "<<theRole.role<<"\n";

    return freeAreas;
  };

/*
@author Emanuele Musumeci
Return a Vector2f containing the chosen target based on the selected mode. There are two modes:
1) the default mode (shootASAP=false) decides wether 
    A) to shoot to the nearest possible target (this happens if the robot 
    has a distance from the opponent goal less or equal than GOAL_TARGET_DISTANCE_THRESHOLD)
    B) to choose the best target based on its utility value
2) the shootASAP mode (shootASAP=true) instead forces shooting to the nearest possible target
*/
//TODO: Move constant parameters to CFG
libCheck.goalTarget = [this](bool shootASAP) -> Vector2f {
    //float GOAL_TARGET_AREA_MIN_SIZE = theBallSpecification.radius*AREA_SIZE_MULTIPLICATOR;
    float GOAL_TARGET_AREA_MIN_SIZE = theOpponentGoalModel.goalTargetAreaMinSize;
    
    float GOAL_TARGET_MIN_OFFSET_FROM_SIDE = theOpponentGoalModel.goalTargetMinOffsetFromSide;
    float GOAL_TARGET_DISTANCE_THRESHOLD = theOpponentGoalModel.goalTargetDistanceThreshold;

    //std::cout<<"TARGET_MIN_OFFSET_FROM_SIDE"<<TARGET_MIN_OFFSET_FROM_SIDE<<"\n";
    //std::cout<<"GOAL_DISTANCE_THRESHOLD="<<GOAL_DISTANCE_THRESHOLD<<"\n";
    //std::cout<<"GOAL_TARGETABLE_AREA_MIN_SIZE="<<GOAL_TARGETABLE_AREA_MIN_SIZE<<"\n";

    std::vector<FreeGoalTargetableArea> freeAreas = computeFreeAreas(GOAL_TARGET_AREA_MIN_SIZE);
    
    /*std::cout<<"FreeAreas: [";
    for(int i=0; i<freeAreas.size(); i++)
    {
      std::cout<<"("<<std::to_string(freeAreas.at(i).end)<<","<<std::to_string(freeAreas.at(i).begin)<<"),";
    }
    std::cout<<"]\n";*/
  
    /*
      1) Filter free areas, ignoring the smaller ones
    */
    std::vector<FreeGoalTargetableArea> filteredFreeAreas;
    for(const auto& area : freeAreas)
    {
      if(area.interval>=GOAL_TARGET_AREA_MIN_SIZE)
      {
        filteredFreeAreas.push_back(area);
        //std::cout << ("[Robot #"+std::to_string(theRobotInfo.number)+"] FreeGoalTargetableArea: ("+std::to_string(area.begin)+","+std::to_string(area.end)+")") << "\n";
      }
    }

    /*
      2) If there is no free area, return the median point on the opponent goal line
    */
    //WATCH OUT: TEMPORARY FIX FOR THE CASE IN WHICH THERE IS NO TARGETABLE AREA, TO MANAGE IN STRIKER BEHAVIOR
    if(filteredFreeAreas.size()==0)
    {
      //std::cout<<"No free area\n";
      
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //return Vector2f(0,0);
      return Vector2f(theFieldDimensions.xPosOpponentGroundline,0);
    } 

    //project my line of view onto the goal line
    //float myGazeProjection = theOpponentGoalModel.myGazeProjection;
    float myGazeProjection = projectGazeOntoOpponentGroundline();
    //std::cout << "myGazeProjection: "+std::to_string(myGazeProjection) << "\n";

    float targetPoint = 0;

    float minTargetableAreaDistance=INFINITY;

    //CASE 1: I'm near the goal post -> choose the nearest free area
    if(shootASAP || (std::abs(theFieldDimensions.xPosOpponentGroundline-theRobotPose.translation.x())<GOAL_TARGET_DISTANCE_THRESHOLD && filteredFreeAreas.size()!=0) || filteredFreeAreas.size()==1)
    {
      for(int i = 0; i < filteredFreeAreas.size(); i++)
      {
        FreeGoalTargetableArea currentArea = filteredFreeAreas.at(i);
        //CASE 1.1: Looking at free area directly
        if(myGazeProjection<currentArea.begin && myGazeProjection>currentArea.end)
        {
          //std::cout<<"Looking at free area ("+std::to_string(currentArea.begin)+","+std::to_string(currentArea.end)+")\n";
          if(myGazeProjection>currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
          else if(myGazeProjection<currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
          else targetPoint = myGazeProjection; 
          break;
        }
        //CASE 1.2: Looking away from free area
        else
        {
          //if freeArea is on the left
          if(currentArea.begin<myGazeProjection)
          { 
            float currentFreeAreaDistance=myGazeProjection-currentArea.begin;
            if(minTargetableAreaDistance>currentFreeAreaDistance)
            {
              minTargetableAreaDistance=currentFreeAreaDistance;
              targetPoint= currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
              //std::cout<<"FreeArea on the left: assigned targetPoint "+std::to_string(targetPoint)+"\n";
            }
          }
          //if freeArea is on the right
          else if(currentArea.end>myGazeProjection)
          {
            float currentFreeAreaDistance=currentArea.end-myGazeProjection;
            if(minTargetableAreaDistance>currentFreeAreaDistance)
            {
              minTargetableAreaDistance=currentFreeAreaDistance;
              targetPoint= currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
              //std::cout<<"FreeArea on the right: assigned targetPoint : "+std::to_string(targetPoint)+"\n";
            }
          }
        }
      }
    }
    //CASE 2: I'm far away from the goal post -> find the area with the highest utility
    else
    {
      //std::cout<< "Seeking best area based on its utility value: theRobotPose.translation.x(): "<< theRobotPose.translation.x()<<", distance from opponentGroundline: "<<std::abs(theRobotPose.translation.x()-theFieldDimensions.xPosOpponentGroundline)<<" > GOAL_DISTANCE_THRESHOLD: "<<GOAL_DISTANCE_THRESHOLD<<"\n";

      //The freeAreas vector is sorted in a decreasing order of utility values, so I select the first area
      targetPoint = filteredFreeAreas.at(0).midpoint;
      
    }
    //std::cout<<"targetPoint y coordinate: "<<targetPoint<<"\n";

    //If I'm too near to the goal line, shoot inside the goal, else shoot on the goal line
    Vector2f target;
    if (theRobotPose.translation.x() >= (theFieldDimensions.xPosOpponentGroundline - 600.f) &&
            std::abs(theRobotPose.translation.y()) < 500.f )
        target = Vector2f(theFieldDimensions.xPosOpponentGroundline + 1000.f, targetPoint);
    else
        target = Vector2f(theFieldDimensions.xPosOpponentGroundline, targetPoint);
      return target;
  };

  libCheck.glob2Rel = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float theta = 0;
      float tempX = x - theRobotPose.translation.x();
      float tempY = y - theRobotPose.translation.y();

      result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
      result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

      return Pose2f(theta , result.x(),result.y());
  };
  
  libCheck.defenderDynamicY = [&]() -> float
  {
        float x2 = theTeamBallModel.position.x();
        float y2 = theTeamBallModel.position.y();
        float x1 = -4500.f;   // first goalpost for defender
        float y1 = (y2/(std::abs(y2)+1))*750.f;   // first goalpost for defender
        float defenderBallY = (( libCheck.defenderPosition.x()-x1 )*( y2-y1 ))/( x2-x1 ) + y1;

        return defenderBallY-(y2/(std::abs(y2)+1))*100.f;
  };
    
  libCheck.rel2Glob = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float rho = (float)(sqrt((x * x) + (y * y)));

      result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
      result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

      return Pose2f(result.x(),result.y());
  };

  libCheck.isValueBalanced = [&](float currentValue, float target, float bound) -> bool
  {
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
  };
  
  libCheck.angleToTarget = [&](float x, float y) -> float
  {
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
  };


  libCheck.getSupporterMarkPosition = [&] () -> Vector2f
  {
    Vector2f strikerPosition;
    Vector2f defenderPosition;

    float STAY_BACK_DISTANCE = 800.f;
    if(theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGroundline + 1500)
      STAY_BACK_DISTANCE = 0;
    else if(theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGroundline / 4.f)
      STAY_BACK_DISTANCE = 350.f;

    for(const auto& teammate : theTeamData.teammates){
        if(teammate.role == Role::RoleType::striker) {
            strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
        else if(teammate.role == Role::RoleType::defender) {
            defenderPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
    }

    Vector2f passPointA = theTeamBallModel.position;

    std::vector<Vector2f> PossibleMarkTargets;
    for(auto const& obs : theTeamPlayersModel.obstacles){
        if(obs.isOpponent() && obs.center.x() < 500.f){
            bool skip = false;
            for(auto const& obs2 : theTeamPlayersModel.obstacles){
                if((obs.center-obs2.center).norm() > 500 && (theRobotPose.translation-obs2.center).norm() > 500.f){
                    // if obs2 is in the line of obs1 and passpointA, this is to be not added to marking
                    if(libCheck.distanceToLine(obs2.center,passPointA,obs.center) < 250.f){
                        float maxDist = (passPointA - obs.center).norm();
                        if((obs2.center-obs.center).norm() < maxDist && (obs2.center-passPointA).norm() < maxDist){
                            skip = true;
                            break;
                        }
                    }
                }
            }
            if((obs.center-passPointA).norm() > 800 && !skip)
                PossibleMarkTargets.emplace_back(obs.center);
        }
    }

    Vector2f leastXAboveStriker(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    Vector2f leastXBelowStriker(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

    for(auto const& obs : PossibleMarkTargets){
        if(obs.y() > strikerPosition.y()){
            if(obs.x() < leastXAboveStriker.x())
                leastXAboveStriker = obs;
        }
        else{
            if(obs.x() < leastXBelowStriker.x())
                leastXBelowStriker = obs;
        }
    }

    // If no mark target
    if(leastXAboveStriker.x() == std::numeric_limits<float>::max() && leastXBelowStriker.x() == std::numeric_limits<float>::max()){
        if(std::abs(strikerPosition.y()) < 500.f){
            if(defenderPosition.y() < strikerPosition.y()){
                float theXPos = (strikerPosition.x()+defenderPosition.x())/2;
                float theYPos = (defenderPosition.y()+strikerPosition.y())/2+1800.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
            else {
                float theXPos = (strikerPosition.x()+defenderPosition.x())/2;
                float theYPos = (defenderPosition.y()+strikerPosition.y())/2-1800.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
        }
        else {
            if(strikerPosition.x() - 500.f > -theFieldDimensions.xPosOpponentGroundline){

                float theXPos = strikerPosition.x() - 500.f;
                float theYPos = strikerPosition.y() - 1800.f;

                if(strikerPosition.y() < 0.f)
                    theYPos = strikerPosition.y() + 1500.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }

                return Vector2f(theXPos,theYPos);
            }
            else {
                float theXPos = strikerPosition.x();
                float theYPos = strikerPosition.y() - 1800.f;

                if(strikerPosition.y() < 0.f)
                    theYPos = strikerPosition.y() + 1500.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
        }
    }
    // Mark the one below striker
    else if(leastXAboveStriker.x() == std::numeric_limits<float>::max()){
        float theXPos = (leastXBelowStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
        float theYPos = (leastXBelowStriker.y()*7+passPointA.y()*3)/10;

        if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
            theXPos += STAY_BACK_DISTANCE;

        if(theXPos > 0.f){
            float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
            theXPos = 0.f;
            theYPos = theYPos * ratio;
        }
        return Vector2f(theXPos,theYPos);
    }
    // Mark the one above striker
    else if(leastXBelowStriker.x() == std::numeric_limits<float>::max()){
        float theXPos = (leastXAboveStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
        float theYPos = (leastXAboveStriker.y()*7+passPointA.y()*3)/10;

        if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
            theXPos += STAY_BACK_DISTANCE;

        if(theXPos > 0.f){
            float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
            theXPos = 0.f;
            theYPos = theYPos * ratio;
        }

        return Vector2f(theXPos,theYPos);
    }
    // Mark the nearest one
    else {
        if((leastXAboveStriker - theRobotPose.translation).norm() < (leastXBelowStriker - theRobotPose.translation).norm()){
            float theXPos = (leastXAboveStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
            float theYPos = (leastXAboveStriker.y()*7+passPointA.y()*3)/10;

            if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
                theXPos += STAY_BACK_DISTANCE;

            if(theXPos > 0.f){
                float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                theXPos = 0.f;
                theYPos = theYPos * ratio;
            }

            return Vector2f(theXPos,theYPos);
        }
        else{
            float theXPos = (leastXBelowStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
            float theYPos = (leastXBelowStriker.y()*7+passPointA.y()*3)/10;

            if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
                theXPos += STAY_BACK_DISTANCE;

            if(theXPos > 0.f){
                float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                theXPos = 0.f;
                theYPos = theYPos * ratio;
            }

            return Vector2f(theXPos,theYPos);
        }
    }
  };

  libCheck.getSupportStrikerPosition = [&] () -> Vector2f
  {
    Vector2f strikerPosition;

    for(const auto& teammate : theTeamData.teammates){
        if(teammate.role == Role::RoleType::striker) {
            strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
    }

    float theXPos = strikerPosition.x() - 600.f;
    float theYPos = strikerPosition.y()/2;
    if(std::abs(theYPos) < 600.f){
        if(theYPos < 0.f)
            theYPos = strikerPosition.y() + 600.f;
        else
            theYPos = strikerPosition.y() - 600.f;
    }

    // if we're defending really close to our goal
    if(strikerPosition.x() < 600.f-theFieldDimensions.xPosOpponentGroundline){
        theXPos = strikerPosition.x() + 200.f;
        if(strikerPosition.y() < 0.f)
            theYPos = strikerPosition.y() + 600.f;
        else
            theYPos = strikerPosition.y() - 600.f;
    }

    // If we're attacking, stay at x=0
    if(theXPos > 0){
        float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
        theXPos = 0;
        theYPos = theYPos * ratio;
    }

    return Vector2f(theXPos,theYPos);

  };

  libCheck.getSupporterPosition = [&] () -> Vector2f
  {
    // if the ball is free
    if(theRoleAndContext.ball_holding_Context == 0){
        float myDistanceToBall = (theRobotPose.translation-theTeamBallModel.position).norm() + 1000.f;
        bool amNearest = true;
        for(const auto& teammate : theTeamData.teammates){
            if(!teammate.isGoalkeeper && teammate.number != theRobotInfo.number) {
                if((teammate.theRobotPose.translation-theTeamBallModel.position).norm() < myDistanceToBall){
                    amNearest = false;
                    break;
                }
            }
        }
        // if(amNearest)
        //     return theTeamBallModel.position;
    }
// std::cout<<"theballmodelpos    "<<theTeamBallModel.position.x()<<"    "<<theTeamBallModel.position.y()<< std::endl;
    Vector2f thePosition;

    if(theRoleAndContext.ball_holding_Context == 2 ||   theRoleAndContext.ball_holding_Context == 3){
        if(theTeamBallModel.position.x() < 0 || libCheck.opponentOnOurField())
            thePosition = libCheck.getSupporterMarkPosition();
        else {
            thePosition = libCheck.getSupportStrikerPosition();
        }
    }
    else{
        if(theTeamBallModel.position.x() < 0 || !libCheck.opponentOnOurField())
            thePosition = libCheck.getSupportStrikerPosition();
        else
            thePosition = libCheck.getSupporterMarkPosition();
    }
    // std::cout<<"thepos    "<<thePosition.x()<<"    "<<thePosition.y()<< std::endl;
    if(!libCheck.obstacleExistsAroundPoint(thePosition))
        return thePosition;
    else {
        if(thePosition.x()-500.f > -theFieldDimensions.xPosOpponentGroundline){
            if(!libCheck.obstacleExistsAroundPoint(Vector2f(thePosition.x()-500.f,thePosition.y())))
                return Vector2f(thePosition.x()-500.f,thePosition.y());
        }
        else if(thePosition.x()+500.f < 0){
            if(!libCheck.obstacleExistsAroundPoint(Vector2f(thePosition.x()-500.f,thePosition.y())))
                return Vector2f(thePosition.x()+500.f,thePosition.y());
        }
        else
            return thePosition;
    }
    return thePosition;
  };

  
    libCheck.radiansToDegree = [&](float x) -> float
  {
    return (float)((x*180)/3.14159265358979323846);
  };

  libCheck.getJollyPosition = [&] () -> Vector2f
  {
    // to return
    Vector2f jollyPosition = Vector2f(0,0);

    // take ball position
    Vector2f ballPosition;
    ballPosition = theTeamBallModel.position;

    // decide the side, should be the opposite
    if(ballPosition.y() < 0.f) {
      if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
        jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 2000.f;
      } else {
        jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 1000.f;
      }
    } else {
      if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
        jollyPosition.y() = theFieldDimensions.yPosRightSideline + 2000.f;
      } else {
        jollyPosition.y() = theFieldDimensions.yPosRightSideline + 1000.f;
      }
    }

    // if the ball is not ahead of the previous jolly position by 1m
    // take previous y
    for(const auto& mate : theTeamData.teammates) {
      if(mate.role == Role::RoleType::jolly) {
        // check jolly position w.r.t. ball
        if(ballPosition.x()+500.f < 0.f) {
          jollyPosition.y() = -1500.f;
        } else if(ballPosition.x() > mate.theRobotPose.translation.x()+1000.f) {
          jollyPosition.y() = mate.theRobotPose.translation.y();
        }
      }
    }

    jollyPosition.x() = theFieldDimensions.xPosOpponentGroundline*0.6f;
    Vector2f translation = Vector2f(0.f, 0.f);
    for(unsigned i=0; i<4; ++i) {
      jollyPosition = jollyPosition+translation;
      // compute best passing line
      Eigen::ParametrizedLine<float,2> toBall = Eigen::ParametrizedLine<float,2>::Through(jollyPosition, ballPosition);

      float minDist = std::numeric_limits<float>::max();
      for(auto obs : theTeamPlayersModel.obstacles) {
        if(obs.center.x() > ballPosition.x() && obs.center.x() < jollyPosition.x()){
          if(obs.isOpponent()) {
            float thisDist = toBall.distance(obs.center);
            if(thisDist < minDist)
              minDist = thisDist;
          }
        }
      }

      if(minDist > 500.f) {
        return jollyPosition;
      }

      // increment translation
      translation.x() = translation.x()-300.f;
    }

    return jollyPosition;
  };

  libCheck.opponentOnOurField = [&] () -> bool
  {
    for(auto const& obs : theTeamPlayersModel.obstacles){
        if(obs.isOpponent() && obs.center.x() < 0.f){
            return true;
        }
    }
    return false;
  };
  
  libCheck.distanceToLine = [&] (Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2) -> float
  {
    return std::abs(((linePoint2.y()-linePoint1.y())*objectToCheck.x()) - ((linePoint2.x() - linePoint1.x()) * objectToCheck.y())
    + (linePoint2.x() * linePoint1.y()) - (linePoint2.y() * linePoint1.x())) / ((linePoint1-linePoint2).norm());
  };

  libCheck.obstacleExistsAroundPoint = [&] (Vector2f point) -> bool
  {
    for(const auto& obstacle : theObstacleModel.obstacles){
        if((obstacle.center - point).norm() < 400.f){
            return true;
        }
    }
    return false;
  };

libCheck.strikerPassShare = [&] () -> std::tuple<int,int,Pose2f>
{
     for(const auto& teammate : theTeamData.teammates){
         if(teammate.role == Role::RoleType::striker) {
             return std::tuple<int,int,Pose2f>(teammate.thePassShare.readyPass,teammate.thePassShare.passingTo,teammate.thePassShare.passTarget);
         }
     }
     return std::tuple<int,int,Pose2f>(0,0,theRobotPose);
};


/**
 * Find a free passing line to the target
 * Returns a valid target or goal center if fails
 *
 * The main function findPassingLine uses and auxiliary function
 * to correctly cycle over all opponents.
 */

  libCheck.findPassingLineAux = [&](std::vector<Obstacle> opponents,
                                          Vector2f& target,
                                          const Vector2f& translation
                                          ) -> bool {
    if(opponents.empty()) {
      target = translation;
      return true;
    }

    // compare against translation
    Eigen::ParametrizedLine<float,2> line = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, translation);

    // in case of failure
    std::vector<Obstacle> originalVector = opponents;

    for(auto it=opponents.begin(); it!=opponents.end(); ++it) {
      if(line.distance(it->center) > 400.f) {
        // call this with reduced obstacles
        opponents.erase(it);
        if(libCheck.findPassingLineAux(opponents, target, translation)) {
          target = translation;
          return true;
        }
      }
      opponents = originalVector;
    }
    return false;
  };

  libCheck.findPassingLine = [&](Vector2f target, std::vector<Vector2f> mates) -> Vector2f {
     bool found = false;
     Vector2f originalTarget = target;
     Vector2f leftTarget = target + Vector2f(0,0);
     Vector2f rightTarget = target - Vector2f(0,0);

     // normalization factor for distance w.r.t. the receiver
     // used to avoid throwing the ball on the back of the receiver
     float nobacknormalization = 1;
     bool isThrowingOnTheBack = false;

     // only use opponents ahead
     // or if behind the opponent penalty mark also use those behind
     std::vector<Obstacle> opponents;
     for(const auto& obs : theTeamPlayersModel.obstacles) {
       if(obs.type == Obstacle::opponent) {
         // if theRobot is inside penalty area then push robots from 2000f to theRobot position
         // else push those ahead only
         if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark &&
            obs.center.x() < theRobotPose.translation.x() &&
            obs.center.x() > 2000.f) {
           opponents.push_back(obs);
         } else if(obs.center.x() > theRobotPose.translation.x() &&
            obs.center.x() < target.x()) {
           opponents.push_back(obs);
         }
       }
     }

     // now cycle over the merged obstacles
     while(!found &&
           leftTarget.y() < theFieldDimensions.yPosLeftSideline &&
           rightTarget.y() > theFieldDimensions.yPosRightSideline) {

       // check if we are throwing the ball on the back of the receiver
       Eigen::ParametrizedLine<float,2> toReceiver1 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, leftTarget);
       Eigen::ParametrizedLine<float,2> toReceiver2 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, rightTarget);
       for(Vector2f mate : mates) {
         // do not do this for back passes
         if(mate.x() < theRobotPose.translation.x()) {
           continue;
         }

         nobacknormalization = 1+(std::abs(mate.x() - theRobotPose.translation.x())/theFieldDimensions.xPosOpponentGroundline);
        if(std::abs(toReceiver1.distance(mate)) < 200.f*nobacknormalization ||
           std::abs(toReceiver2.distance(mate)) < 200.f*nobacknormalization) {
           isThrowingOnTheBack = true;
           break;
         }
       }

       if(isThrowingOnTheBack) {
         // increase left and right
         isThrowingOnTheBack = false;
       } else {
         // compute the actual passage
         if(target.y() > theRobotPose.translation.y()) {
           if(libCheck.findPassingLineAux(opponents, target, rightTarget) ||
              libCheck.findPassingLineAux(opponents, target, leftTarget)) {
             return target;
           }
         } else {
           if(libCheck.findPassingLineAux(opponents, target, leftTarget) ||
              libCheck.findPassingLineAux(opponents, target, rightTarget)) {
             return target;
           }
         }
       }

       // increase left and right
       leftTarget = leftTarget + Vector2f(0,50);
       rightTarget = rightTarget - Vector2f(0,50);
     }
     // maybe next time
     return Vector2f(theFieldDimensions.xPosOpponentGroundline, 0);
  };

/*
 * This function orderes the team mates according to their distance w.r.t. the opponents
 * and then, starting from the most free one, tries to find a passing line
 * @return a Vector2f poiting to the passing target or the goal line if no passing is found
 */
  libCheck.poseToPass = [&]() -> Pose2f
  {
    // custom team mate definition
    struct eMate {
      Vector2f position;
      float utility;
      int number;
    };

    // generate the list of valid teammates
    std::vector<eMate> orderedMates;
    // generate auxiliary list of vector2f of available mates
    std::vector<Vector2f> auxMates;
    // for rear pass do not consider forward passage, do this only for forward pass
    float passForward = 600.f;

    for(const auto& mate : theTeamData.teammates) {
      // enable rear passage if inside the opponent area
      // or if not robots have to be ahead of us
      if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark && mate.theRobotPose.translation.x() > 2000.f)
         || mate.theRobotPose.translation.x() > theRobotPose.translation.x()) {
        eMate emate;
        emate.position = mate.theRobotPose.translation;
        emate.utility = std::numeric_limits<float>::max();
        emate.number = mate.number;

        // order w.r.t. distance from the closest opponent
        for(const auto& obs : theTeamPlayersModel.obstacles) {
          if(obs.type == Obstacle::opponent &&
             obs.center.x() <= mate.theRobotPose.translation.x()) {
            // compute emate utility w.r.t. obs
            float newUtility = (float)(std::pow(emate.position.x()-obs.center.x(),2) +
              std::pow(emate.position.y()-obs.center.y(),2));
            if(newUtility < emate.utility) {
              emate.utility = newUtility;
            }
          }
        }
        // order w.r.t utility
        bool added = false;
        if(orderedMates.empty()) {
          orderedMates.push_back(emate);
          auxMates.push_back(mate.theRobotPose.translation);
        } else {
          for(auto it = orderedMates.begin(); it != orderedMates.end(); ++it) {
            if(it->utility < emate.utility) {
              orderedMates.insert(it, emate);
              added = true;
              break;
            }
          }
        }
        if(!added) {
          // actual ordered list of mates
          // TODO remove to optimize
          orderedMates.push_back(emate);
          // use for the chek of robot backs (see find passing line)
          auxMates.push_back(mate.theRobotPose.translation);
        }
      }
    }

    // now call the findPassingLine starting from the best mate
    for(const auto& mate : orderedMates) {
      if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
        passForward = 0.f;
      }
      Vector2f target = libCheck.findPassingLine(mate.position, auxMates);
      // make forward pass
      if(mate.position.x() < theFieldDimensions.xPosOpponentPenaltyMark) {
        // increase the x
        target.x() = target.x()+passForward;
      }

      // if target is the center of the goal then no valid passing line, continue
      if(target != Vector2f(theFieldDimensions.xPosOpponentGroundline+passForward,0.f)){
        libCheck.isTargetToPass = mate.number;
        return target;
      }
    }
    return Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f);
  };

  libCheck.canPass = [&](Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle> opponents) -> bool
  {
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
    //TODO inserire il vettore degli opponents
    for(auto const& opponent : opponents){
      if(sameX){
        if(std::abs(opponent.center.x() - shootingPose.translation.x()) < qThreshold){
          std::cout<<"1"<<std::endl;
          return false;
        }
      }
      else if(sameY){
        if(std::abs(opponent.center.y() - shootingPose.translation.y()) < qThreshold){
          std::cout<<"2"<<std::endl;
          return false;
        }
      }
      else{
        for(int i = -50; i < 50; i++){
          //se l'opponent si trova su una retta del fascio
          if((opponent.center.y() - m * opponent.center.x() - q - (float) i * 10.f) <= 0.5f ){
            std::cout<<"3"<<std::endl;
            return false;
          }
        }
      }
    }
    return true;

  };

  
  libCheck.defenderPosition = updateDefender();
  libCheck.supporterPosition = updateSupporter();
  libCheck.goaliePosition = updateGoalie();
  libCheck.jollyPosition = Vector2f(theFieldDimensions.yPosLeftGoal, theFieldDimensions.yPosRightGoal);

  libCheck.initialize_PF = [&](float cell_size) -> std::vector<NodePF>
  {
    std::vector<NodePF> potential_field;
    //init obstacle model
    int d = cell_size;
    for(int i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= d) //TODO parametrize
    {
        for(int j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= d)
        {
            potential_field.push_back( NodePF(Vector2f( i-d/2,j-d/2 ), Vector2f()) );
            if(i==SPQR::FIELD_DIMENSION_X - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(i== -SPQR::FIELD_DIMENSION_X + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(j== SPQR::FIELD_DIMENSION_Y - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(j== -SPQR::FIELD_DIMENSION_Y + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(i>=SPQR::FIELD_DIMENSION_X-(SPQR::FIELD_DIMENSION_X/50) && j<=800 && j >= -800) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
        }
    }
    return potential_field;
  };

  libCheck.compute_striker_attractive_PF = [&] (Vector2f goal, float RO = 1000.f, 
                                                    float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f, 
                                                    float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f) -> std::vector<NodePF>
  {

      //Attractive potential field toward the center of the soccer field + Repulsive potential field written as an opposite-attractive component
      std::vector<NodePF> attractive_field(theStrikerPFModel.potential_field.size());

      for(unsigned int i=0; i<attractive_field.size(); ++i)
      {
        Vector2f current_cell_pos = theStrikerPFModel.potential_field.at(i).position;

        Vector2f tmp_err = goal - current_cell_pos;

        if( tmp_err.norm() <= RO)
        {
          Vector2f conic_field = Vector2f(Kap * tmp_err.x(), Kap * tmp_err.y());
          attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),conic_field);
        }
        else
        {
          Vector2f quadratic_field = Vector2f((Kbp/tmp_err.norm()) * tmp_err.x(), (Kbp/tmp_err.norm()) * tmp_err.y());
          attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),quadratic_field);
        }
      }

      return attractive_field;
  };

  libCheck.compute_striker_repulsive_PF = [&](float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f, 
                                                    float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f) -> std::vector<NodePF>
  {

    Vector2f my_pos = theRobotPose.translation;

    //Build a vector out of all obstacles, including OPPONENTS and TEAMMATES, excluding POLES
    std::vector<Vector2f> repulsive_obstacles;    
    for(auto obs : theTeamPlayersModel.obstacles)
    {
      /*NOTICE: poles are added statically (a priori) by the vision system
        so the 4 goal poles will always be in the obstacle list*/
      switch(obs.type)
      {
        case Obstacle::Type::goalpost:
        {
          break;
        }
        default:
        {
          repulsive_obstacles.push_back(obs.center);
          break;
        }
      }
    }

    //Repulsive potential field from other robots and/or regions of the field
    std::vector<NodePF> repulsive_field(theStrikerPFModel.potential_field.size());

    for(unsigned int i=0; i<repulsive_field.size(); ++i)
    {
        Vector2f current_cell_pos = theStrikerPFModel.potential_field.at(i).position;
      
        repulsive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(),current_cell_pos.y()), Vector2f(0,0));

        for(unsigned int r=0; r<repulsive_obstacles.size(); ++r)
        {

            if(r == (uint) theRobotInfo.number-1) continue;
            if (repulsive_obstacles.at(r).x() == 0 || repulsive_obstacles.at(r).y() == 0) continue;

            Vector2f tmp_err = repulsive_obstacles.at(r) - current_cell_pos;
            if( (tmp_err).norm() < ETA)
            {
                float tmp_eta = (repulsive_obstacles.at(r)-my_pos).norm();

                Vector2f tmp_rep = repulsive_obstacles.at(r)-my_pos;


                repulsive_field.at(i).potential += Vector2f( -tmp_rep * (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (1/tmp_err.norm()));
                /// original else bbody
                /// repulsive_field.at(i)  += (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (-tmp_rep*(1/tmp_err.translation.absFloat()));

            }
            else
            {
                repulsive_field.at(i).potential += Vector2f(0.f, 0.f);
            }
        }
    }

    return repulsive_field;
  };

  libCheck.computePF = [&](std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field, float cell_size) -> std::vector<NodePF>
  {

    std::vector<NodePF> potential_field = initialize_PF(cell_size);

  ///////////////////////////////
  //NOTICE: can be parallelized//
  ///////////////////////////////

    for(unsigned int p=0; p<potential_field.size(); ++p)
    {
      potential_field.at(p).potential = (attractive_field.at(p).potential + repulsive_field.at(p).potential);
    }

    //attractive_field.clear();
    //repulsive_field.clear();
    //potential_field.clear();

    return potential_field;
  };
  libCheck.angleForDefender  = angleToTarget(libCheck.defenderPosition.x(), libCheck.defenderPosition.y());
  libCheck.angleForSupporter = angleToTarget(libCheck.supporterPosition.x(), libCheck.supporterPosition.y());
  //libCheck.angleForJolly = angleToTarget(libCheck.jollyPosition.x(), libCheck.jollyPosition.y());
  libCheck.angleForJolly = angleToTarget(theFieldDimensions.yPosLeftGoal, theFieldDimensions.yPosRightGoal);//check this part the values of YposleftGoal and soon

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////END OF UPDATE////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void LibCheckProvider::reset()
{
  FOREACH_ENUM(LibCheck::CheckedOutput, i)
    callCounters[i] = 0;

  for(int i = 0; i < Arms::numOfArms; ++i)
    setArmsInThisFrame[i] = false;
}

void LibCheckProvider::checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const
{
  const std::string options = getActivationGraphString(activationGraph);

  // Output counting checks:
  for(LibCheck::CheckedOutput i = start; i < end; i = static_cast<LibCheck::CheckedOutput>(static_cast<unsigned>(i) + 1))
  {
    // Check, if output has been set at least once:
    if(callCounters[i] == 0 && notSetCheck[i] == 1)
    {
      OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(i) << " has not been set in this cycle (Robot " << theRobotInfo.number
                  << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                  << (options == "" ? "" : ", Options: " + options) << ") !");
    }
    else if(notSetCheck[i] == 2)
    {
      ASSERT(callCounters[i] > 0);
    }
  }
}


void LibCheckProvider::checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const
{
  // Check for invalid motion request:
  if(assertValidWalkRequest &&
     theMotionRequest.motion == MotionRequest::walk &&
     !theMotionRequest.walkRequest.isValid())
  {
#ifndef NDEBUG
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "walkRequest.log");
      stream << theMotionRequest.walkRequest;
      stream << getActivationGraphString(activationGraph);
    }
#endif
    FAIL("Motion request is not valid (see walkRequest.log).");
  }
}

void LibCheckProvider::inc(LibCheck::CheckedOutput outputToCheck)
{
  const int index = static_cast<int>(outputToCheck);
  if(index >= 0 && index < LibCheck::numOfCheckedOutputs)
  {
    ++callCounters[index];

    // Check, if output has not been set more than once:
    if(callCounters[index] > 1)
    {
      if(multipleSetCheck[index] == 1)
      {
        const std::string options = getActivationGraphString(index >= LibCheck::firstTeamCheckedOutput ? theTeamActivationGraph : theActivationGraph);

        OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(static_cast<LibCheck::CheckedOutput>(index)) << " has been set more than once in this cycle (Robot "
                    << theRobotInfo.number
                    << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                    << (options == "" ? "" : ", Options: " + options) << ") !");
      }
      else if(multipleSetCheck[index] == 2)
      {
        ASSERT(callCounters[index] <= 1);
      }
    }
  }
}

Pose2f LibCheckProvider::myReadyPosition() const{
  Pose2f strikerPose = Pose2f(0.f, -1000.f, 0.f);
  if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
    strikerPose = Pose2f(0.f, -500.f, 0.f);
  }else{
    strikerPose = Pose2f(0.f, -1000.f, 0.f);
  }
  
  Pose2f goaliePose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 200.f, 0.f);
  Pose2f defenderPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1000.f, -1000.f);
  Pose2f jollyPose = Pose2f(0.f, -500.f, -1500.f);
  Pose2f supporterPose = Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 1700.f, 800.f);
  int activeTeam = 0;
  int lowerNumbers = 0;
  if(theRobotInfo.penalty == PENALTY_NONE){
    activeTeam++;
  }
  if(theRobotInfo.number == 1){
      return goaliePose;
  }
  for( auto teammate : theTeamData.teammates ){
    if(teammate.status != Teammate::PENALIZED && teammate.number != 1){
      activeTeam ++;
      if(teammate.number < theRobotInfo.number){
        lowerNumbers ++;
      }
    }
  }
  switch(activeTeam){
    case 0: return Pose2f(0.f,0.f,0.f); break;

    case 1: return strikerPose;
            break;

    case 2: if(lowerNumbers <= 0){
              return defenderPose;
            }else{
              return strikerPose;
            } 
            break;

    case 3: if(lowerNumbers == 2){
                return strikerPose;
              }else if(lowerNumbers == 1){
                return defenderPose;
              }else if(lowerNumbers == 0){
                return supporterPose;
              }
              break;

    case 4: if(theRobotInfo.number == 3){
              return strikerPose;
            }else if(theRobotInfo.number == 2){
              return supporterPose;
            }else if (theRobotInfo.number == 5){
              return defenderPose;
            }else if(theRobotInfo.number == 4){
              return jollyPose;
            }
  }

  return strikerPose;

}

float LibCheckProvider::distance(float x1, float y1, float x2, float y2) const{

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));

}

float LibCheckProvider::distance(Pose2f p1, Pose2f p2) const{

  return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) + 
    std::pow(p2.translation.y() - p1.translation.y(), 2) ) );

}

std::string LibCheckProvider::getActivationGraphString(const ActivationGraph& activationGraph) const
{
  std::string options = "";
  for(const auto& node : activationGraph.graph)
    options += (options == "" ? "" : ", ") + node.option + (node.state == "" ? "" : "/" + node.state);
  return options;
}

Pose2f LibCheckProvider::refineTarget(Pose2f t, float d){
  
  Rangef zeroTreshold = Rangef({-0.1f, 0.1f});
  float diffX = t.translation.x() - theRobotPose.translation.x();
  float diffY = t.translation.y() - theRobotPose.translation.y();

  //Check if target and robot have the same X
  if(zeroTreshold.isInside(diffX)){
    //if they also have the same y keep the same target
    if(zeroTreshold.isInside(diffY)){
      return t;
    }else{
      return Pose2f(theRobotPose.translation.x(), theRobotPose.translation.y() + (d * sign(diffY) ));
    }
  }// end of if X1 == X2

  //Check if target and robot have the same Y
  if(zeroTreshold.isInside(diffY)){
    //if they also have the same x keep the same target
    if(zeroTreshold.isInside(diffX)){
      return t;
    }else{
      return Pose2f( theRobotPose.translation.x() + (d * sign(diffX)) , theRobotPose.translation.y());
    }
  }// end of if Y1 == Y2

  //if the distance is still lower than d, than just return t
  if(distance(theRobotPose, t) <= d ){
    return t;
  }

  // Y = mX + q
  float m = diffY/diffX;
  float q = -theRobotPose.translation.y() -theRobotPose.translation.x()*m;  
  // just for readability
  float x1 = theRobotPose.translation.x();
  float y1 = theRobotPose.translation.y();
  
  float a = SQ(m) + 1;
  float b = 2*m -2*y1*m -2*x1;
  float c = SQ(q) -2*y1*q +SQ(y1) +SQ(x1) -SQ(d);
  
  //Use the delta formula
  float X = (-b + std::sqrt(SQ(b) - 4*a*c))/(2*a);
  //classical line equation
  float Y = m*X + q;

  return Pose2f(X,Y);

}

//@author Emanuele Musumeci
//Project my line of sight (the direction i'm looking at) on the opponent ground line
float LibCheckProvider::projectGazeOntoOpponentGroundline()
{
    return theRobotPose.translation.y() + std::abs(theFieldDimensions.xPosOpponentGoal - theRobotPose.translation.x())*tan(theRobotPose.rotation);
}

//@author Emanuele Musumeci
//Project a point on the opponent ground line
float LibCheckProvider::projectPointOntoOpponentGroundline(float x, float y)
{
  return ((theFieldDimensions.xPosOpponentGroundline - theRobotPose.translation.x())/(x - theRobotPose.translation.x()))*(y - theRobotPose.translation.y()) + theRobotPose.translation.y();
};

//Questa è quella eseguita veramente
// 1) Note: We are considering only the closest obstacle to the freeArea. 
float LibCheckProvider::areaValueHeuristic(const float leftLimit, const float rightLimit) 
{
  // Useful notes for the reader:
  //       leftLimit is the left limit of THIS free area, rightLimit is the right one. 
  //       Note that the axis is directed to the left, then leftLimit > righLimit.

    float BASE_UTILITY = theOpponentGoalModel.baseUtility;
    std::vector<float> opponents_distances;
    std::vector<float> teammates_distances;
    
    Pose2f freeAreaPoseleftLimit= Pose2f(theFieldDimensions.xPosOpponentGroundline,leftLimit);
    Pose2f freeAreaPoserightLimit= Pose2f(theFieldDimensions.xPosOpponentGroundline,rightLimit);

    for(auto obs : theTeamPlayersModel.obstacles){
        
        if ((obs.center.x()==theFieldDimensions.xPosOpponentGoalPost) && (obs.center.y()==theFieldDimensions.yPosLeftGoal || obs.center.y()==theFieldDimensions.yPosRightGoal)){
          continue;  // i pali non li conto
        }
        if ((obs.center.x()==-theFieldDimensions.xPosOpponentGoalPost) && (obs.center.y()==theFieldDimensions.yPosLeftGoal || obs.center.y()==theFieldDimensions.yPosRightGoal)){
          continue;  // i pali non li conto
        }
        if(theRobotPose.translation.x()>obs.center.x()){
          continue; // if the obstacle is back wrt me, it is not considered
        }

        float final_distance=0;

        //###########     Considering projection wrt my eyes 

   
        float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
        float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
        float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());


        // IT'S NOT POSSIBLE THAT THIS PROJECTION IS INTO SOME FREE_AREAS. Then, just check if it is left or right wrt the obstacle.

        if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
          final_distance=obs_right_proj-leftLimit;

        }

        if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
          final_distance=rightLimit-obs_left_proj;
        }

        // Differentiate the obstacles in two sets: one of oppontents, one of teammates.
        if(obs.isOpponent()){
          opponents_distances.push_back(final_distance);
        }
        if(obs.isTeammate()){
          teammates_distances.push_back(final_distance);
        }      
       

    }

    // Although they are in two different vectors, they are treated in the same way. 
    // However if one in the future wants to use this feature, it is ready to be exploited.
    // Let's consider just the closest opponent
    float minimo_opponents=0;
    bool firstTime=true;
    for(auto distance : opponents_distances){
      if(firstTime){
        firstTime=false;
        minimo_opponents=distance;
      }else if(minimo_opponents>distance){
        minimo_opponents=distance;
      }
    }

    //Let's use the same minimum variable for both opponents and teammetes ( no distinction )
    float minimo_teammates=0;
    firstTime=false;
    for(auto distance : teammates_distances){
      if(firstTime){
        firstTime=false;
        minimo_opponents=distance;
      }
      if(minimo_opponents>distance){
        minimo_opponents=distance;
      }
    }
    // final utility is always positive.
    float final_utility=BASE_UTILITY+minimo_opponents+minimo_teammates;

    return final_utility;

    //Ideas that can be exploited in order to improve this function:
    /*
    1) First criteria: vicinity to jolly (or in alternative the second robot nearest to the ball, as the first one is the striker)
    2) Second criteria: number of opponent in intercept range
    3) Third criteria: vicinity of enemy goalie (is this possible?)
    4) Width of parent area
    5) Penalize the poles, but this can be done at an higher level of abstraction
    */
  };


  //RECENTLY ADDED
  bool LibCheckProvider::areOverlappingSegmentsOnYAxis(float l1, float r1, float l2, float r2)
  {
    return ((l1<l2 && l1>r2) || (r1<l2 && r1>r2) || (l2<l1 && l2>r1) || (r2<l1 && r2>r1));
  };

  std::vector<FreeGoalTargetableArea> LibCheckProvider::computeFreeAreas(float minimumDiscretizedAreaSize)
  {
    /*
    NOTICE:
    begin = leftLimit
    end = rightLimit
    */
  
    Pose2f myPose = Pose2f(theRobotPose.translation);

    float GOAL_TARGET_OBSTACLE_INFLATION = theOpponentGoalModel.goalTargetObstacleInflation;
    float GOAL_TARGET_AREA_DISCRETIZATION = theOpponentGoalModel.useAreaDiscretization;

    //vector of opponents
    std::vector<Obstacle> opponents;
    //vector of poles
    std::vector<Obstacle> poles;

    for(auto obs : theTeamPlayersModel.obstacles){
      /*NOTICE: poles are added statically (a priori) by the vision system
        so the 4 goal poles will always be in the obstacle list*/
      switch(obs.type)
      {
        case Obstacle::Type::goalpost:
        {
          if(obs.type==Obstacle::Type::goalpost && obs.center.x()>0)
          {
            //std::cout<<"Found opponent goal post: (left:"<<obs.left.y()<<", right:"<<obs.right.y()<<")\n";
            poles.push_back(obs);
          }
          break;
        }
        default:
        {
          if(obs.center.x()>theRobotPose.translation.x() || obs.left.x()>theRobotPose.translation.x() || obs.right.x()>theRobotPose.translation.x())
          {
            opponents.push_back(obs);
          }
          break;
        }
      }
    }

    Pose2f leftPole, rightPole;

    if(poles.size()==0)
    {
      leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),730.);
      if(leftPole.translation.y()>730) leftPole.translation.y()=730; 
      rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),-730.);
      if(rightPole.translation.y()<-730) leftPole.translation.y()=-730; 
    }
    else
    {
      
      leftPole.translation.x() = poles.at(0).right.x();
      if(leftPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundline) leftPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundline; 

      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),poles.at(0).right.y());
      if(leftPole.translation.y()>730) leftPole.translation.y()=730; 

      rightPole.translation.x() = poles.at(1).left.x();
      if(rightPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundline) rightPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundline; 

      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),poles.at(1).left.y());
      if(rightPole.translation.y()<-730) rightPole.translation.y()=-730; 
      
      /*
      WORK IN PROGRESS - Trying to use the projection of the poles on the groundline as left/right limit of the
      goal line (useful in case the player is particularly angled wrt the opponent goal)
      
      Vector2f leftPoleRightLimit(
        1/tan(theRobotPose.rotation+asin(POLE_RADIUS/distance(theRobotPose,poles.at(0).center))),
        poles.at(0).center.y() - tan(theRobotPose.rotation+asin(POLE_RADIUS/distance(theRobotPose,poles.at(0).center)))
      );
      leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),leftPoleRightLimit.y());
      
      Vector2f rightPoleLeftLimit(
        1/tan(theRobotPose.rotation-asin(POLE_RADIUS/distance(theRobotPose,poles.at(1).center))),
        poles.at(1).center.y() - tan(theRobotPose.rotation-asin(POLE_RADIUS/distance(theRobotPose,poles.at(1).center)))
      );
      rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
      rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),rightPoleLeftLimit.y());
      */

      //std::cout<<"Left goal pole projection: ("<<leftPole.translation.x()<<","<<leftPole.translation.y()<<")\n";
      //std::cout<<"Right goal pole projection:"<<rightPole.translation.x()<<","<<rightPole.translation.y()<<")\n";
    }
    
    //FREE AREAS COMPUTATION

    std::vector<float> leftPoints;
    std::vector<float> rightPoints;
    std::vector<FreeGoalTargetableArea> freeAreas;

    
    Obstacle swapper;

    /*1) Sort the opponents in vector based on the y coordinate of their left points 
    (for each obstacle, the leftmost point I see)*/
    for(int i = 0; i < opponents.size(); i++){
        for(int k = 0; k < opponents.size(); k++){
            float firstLeft = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y());
            float secondLeft = projectPointOntoOpponentGroundline(opponents.at(k).left.x(),opponents.at(k).left.y());
            if(firstLeft > secondLeft ){
                swapper = opponents.at(k);
                opponents.at(k) = opponents.at(i);
                opponents.at(i) = swapper;
            }
        }
    }
    /*std::cout<<"Opponents: [";
    for(int i=0;i<opponents.size();i++)
    {
      std::cout<<"("<<std::to_string(opponents.at(i).center.x())+","<<opponents.at(i).center.y()<<"), ";
    }
    std::cout<<"]\n";*/

    /*2) Find overlapping obstacles and merge them into a single one. For each obstacle (including the
        ones obtained by merging) save the projections on the goal line of its left and right point, in order
        to populate a list of obstacles from the player's point of view.

        NOTICE: the obstacles are ordered from left to right

        NOTICE: GOAL_TARGET_OBSTACLE_INFLATION is used as an "obstacle inflation" factor
        to enlarge obstacles (as the visualization system makes the opponent robots smaller than their
        feet width
    */
    
    if(opponents.size()>0)
    {
      float leftPointY = projectPointOntoOpponentGroundline(opponents.at(0).left.x(),opponents.at(0).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(0).left);
      float rightPointY = projectPointOntoOpponentGroundline(opponents.at(0).right.x(),opponents.at(0).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(0).right);
      if(opponents.size()==1)
      {
        //If the obstacle projection is at least partially inside the goal line add it
        if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
        {
          //std::cout<<"1 Adding single obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
          leftPoints.push_back(leftPointY);
          rightPoints.push_back(rightPointY);
        }
      }
      else if(opponents.size()>1)
      {
        int i=1;
        bool wereOverlapping;

        /*
          For each obstacle in the list, compare it against the next one to check for overlapping and,
          depending on the kind of overlapping, merge them accordingly or add them separately
        */
        while(i<opponents.size()+1)
        {
          float nextLeftPointY, nextRightPointY;
          if(i==opponents.size())
          {
            /*
              If leftPoint and rightPoint identify the last obstacle in the opponents list, use 
              the right pole as the next obstacle to compare for overlapping
            */
            nextLeftPointY = projectPointOntoOpponentGroundline(poles.at(1).left.x(),poles.at(1).left.y());
            nextRightPointY = projectPointOntoOpponentGroundline(poles.at(1).right.x(),poles.at(1).right.y());
          }
          else
          {
            /*
              Else use the next obstacle in the list
            */
            nextLeftPointY = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(i).left);
            nextRightPointY = projectPointOntoOpponentGroundline(opponents.at(i).right.x(),opponents.at(i).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/distance(theRobotPose,opponents.at(i).right);
          }
          //std::cout<<"Iteration #"<<i<<": LeftPointY: "+std::to_string(leftPointY)+", RightPointY: "+std::to_string(rightPointY)+"\n";
          //std::cout<<"Iteration #"<<i<<": nextLeftPointY: "+std::to_string(nextLeftPointY)+", nextRightPointY: "+std::to_string(nextRightPointY)+"\n";
          
          /*
            Check for overlapping: there are three cases to manage
            1) One obstacle is inside the other: do nothing
            2) Obstacles overlap only partially: merge the obstacles
            3) No overlap: add the first obstacle and pass to the second one
          */
          if(areOverlappingSegmentsOnYAxis(leftPointY, rightPointY, nextLeftPointY, nextRightPointY))
          {
            //std::cout<<"Overlapping: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+") with ("+std::to_string(nextLeftPointY)+","+std::to_string(nextRightPointY)+")\n";
            
            if(leftPointY>nextLeftPointY && rightPointY<nextRightPointY)
            {
              //1) One obstacle is inside the other: do nothing

              //std::cout<<"CASE 1\n";
              //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
            }
            else
            {
              //2) Obstacles overlap only partially: merge the obstacles

              //std::cout<<"CASE 2\n";
              rightPointY = nextRightPointY;
              
              //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  

            }
            wereOverlapping=true;
          }
          else
          {  
            //3) No overlap: add the first obstacle and pass to the second one
            if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
            {
              //Add the obstacle projection only if it falls (even only partially) inside the goal line
              //std::cout<<"2 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
              leftPoints.push_back(leftPointY);
              rightPoints.push_back(rightPointY);
            }
            leftPointY = nextLeftPointY;
            rightPointY = nextRightPointY;
            wereOverlapping=false;
          }

          i++;
        }

        //Add last remaining obstacle points (only in case where it overlaps right pole)
        if(wereOverlapping)
        {  
          if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
          {
            //Add the obstacle projection only if it falls (even only partially) inside the goal line
            //std::cout<<"3 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";  
            leftPoints.push_back(leftPointY);
            rightPoints.push_back(rightPointY);
          }
        }      
      }
    }

    //std::cout<<"End of overlap check\n\n";
    
    /*3) Now that we have left and right points of the obstacles
      3.1) We first determine if there is any free area (or if a single obstacle 
           projection overlaps the whole goal line)
      3.2) We shrink the left and right limit (begin/end) of the goal line if there are
           obstacles overlapping the left/right poles
    */
    //determines if all obstacles are outside goal
    bool noneInside = true;

    //Consider my angle in inflating the pole
    float begin = leftPole.translation.y();
    float end = rightPole.translation.y();

    for(int i = 0; i < leftPoints.size(); i++){

        //3.1)
        //at least one point inside goal
        if(leftPoints.at(i)<leftPole.translation.y() && rightPoints.at(i)>rightPole.translation.y() || 
        leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i)<leftPole.translation.y() || 
        leftPoints.at(i)>rightPole.translation.y() && rightPoints.at(i)<rightPole.translation.y())
        {
          noneInside = false;
        }
//WATCH OUT, TEMPORARY SOLUTION: If an obstacle projection cover the whole goal area I return an empty area list
        if(leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i) < rightPole.translation.y())
        {
          return freeAreas;
        }

        //3.2)
        //left obstacle border outside goal, right border inside
        if(leftPoints.at(i) > leftPole.translation.y() && rightPoints.at(i) < leftPole.translation.y()){
            begin = rightPoints.at(i);
        }
        //right obstacle border outside goal, left border inside
        if(leftPoints.at(i) > rightPole.translation.y() && rightPoints.at(i) < rightPole.translation.y()){
            end = leftPoints.at(i);
        }
    }

    /*4) Build the free targetable segments vector
      4.1) If there is no targetable segment, return the empty vector
      4.2) Else populate the freeAreas vector
    */

    //4.1)
    if(noneInside == true){
      freeAreas.push_back(FreeGoalTargetableArea(begin, end, 1));
      //std::cout<<"NONE INSIDE\n";
        return freeAreas;
    }
    std::vector<float> freeAreasPoints;
    freeAreasPoints.push_back(begin);

    //4.2)
    for(int i = 0; i < leftPoints.size(); i++){
        if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
            freeAreasPoints.push_back(leftPoints.at(i));
        }
        if(rightPoints.at(i) < begin && rightPoints.at(i) > end ){
            freeAreasPoints.push_back(rightPoints.at(i));
        }
    }
    freeAreasPoints.push_back(end);

    sort(freeAreasPoints.begin(),freeAreasPoints.end());

    /*std::cout<<"FreeAreasPoints: [";
    for(int i=0;i<freeAreasPoints.size();i++)
    {
      std::cout<<std::to_string(freeAreasPoints.at(i))+",";
    }
    std::cout<<"]\n";*/

    /*5A) Apply discretization to the targetable segments, by dividing targetable areas in smaller segments,
        in order to assign them a utility value
    */

    if(GOAL_TARGET_AREA_DISCRETIZATION)
    {
      for(int i = freeAreasPoints.size()-1; i-1>=0; i-=2)
      {
        float beginning = freeAreasPoints.at(i);
        float end = freeAreasPoints.at(i-1);
        float size = std::abs(end-beginning);
        if(size>minimumDiscretizedAreaSize)
        {
          int numberOfDiscretizedAreasInFreeArea = floor(size/minimumDiscretizedAreaSize);

          float discretizedAreaSize = size/numberOfDiscretizedAreasInFreeArea;

          float firstPoint = beginning;
          for(int i = numberOfDiscretizedAreasInFreeArea -1; i>0; i--)
          {
            float lastPoint = firstPoint-discretizedAreaSize;
            freeAreas.push_back(FreeGoalTargetableArea(firstPoint,lastPoint,areaValueHeuristic(firstPoint,lastPoint)));
            firstPoint = lastPoint;
          }
            freeAreas.push_back(FreeGoalTargetableArea(firstPoint,end,areaValueHeuristic(firstPoint,end)));
        }
        else
        {
          continue;
        }
        
      }

      /*
        6) Sort the result of the discretization on its utility value
      */

      //Sort the freeAreas vector in a decreasing order of utility values
      sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
      /*for(const auto& free: freeAreas)
      {
        std::cout<<free.value<<" ";
      }
      std::cout<<"\n";*/
    }
    else
    {
      /*
        5B) Do not discretize free areas: the whole area between opponent projections is used:
            might be imprecise and less effective and also will require longer times to align with the ball
      */
      for(int i = freeAreasPoints.size() -1; i-1 >= 0; i-=2)
      {
        freeAreas.push_back(FreeGoalTargetableArea(freeAreasPoints.at(i), freeAreasPoints.at(i-1), areaValueHeuristic(freeAreasPoints.at(i),freeAreasPoints.at(i-1))));
      }
      sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
    }
    
    /*std::cout<<"Discretized FreeAreas: [";
    for(int i=0; i<freeAreas.size(); i++)
    {
      std::cout<<"("<<std::to_string(freeAreas.at(i).end)<<","<<std::to_string(freeAreas.at(i).begin)<<"),";
    }
    std::cout<<"]\n";*/

    //std::cout<<"Role: "<<theRole.role<<"\n";

    return freeAreas;
  }

//RECENTLY MODIFIED
//TODO: Move constant parameters to CFG
Vector2f LibCheckProvider::goalTarget(bool shootASAP)
{
    //float GOAL_TARGET_AREA_MIN_SIZE = theBallSpecification.radius*AREA_SIZE_MULTIPLICATOR;
    float GOAL_TARGET_AREA_MIN_SIZE = theOpponentGoalModel.goalTargetAreaMinSize;
    
    float GOAL_TARGET_MIN_OFFSET_FROM_SIDE = theOpponentGoalModel.goalTargetMinOffsetFromSide;
    float GOAL_TARGET_DISTANCE_THRESHOLD = theOpponentGoalModel.goalTargetDistanceThreshold;

    //std::cout<<"TARGET_MIN_OFFSET_FROM_SIDE"<<TARGET_MIN_OFFSET_FROM_SIDE<<"\n";
    //std::cout<<"GOAL_DISTANCE_THRESHOLD="<<GOAL_DISTANCE_THRESHOLD<<"\n";
    //std::cout<<"GOAL_TARGETABLE_AREA_MIN_SIZE="<<GOAL_TARGETABLE_AREA_MIN_SIZE<<"\n";

    std::vector<FreeGoalTargetableArea> freeAreas = computeFreeAreas(GOAL_TARGET_AREA_MIN_SIZE);
    
    /*std::cout<<"FreeAreas: [";
    for(int i=0; i<freeAreas.size(); i++)
    {
      std::cout<<"("<<std::to_string(freeAreas.at(i).end)<<","<<std::to_string(freeAreas.at(i).begin)<<"),";
    }
    std::cout<<"]\n";*/
  
    /*
      1) Filter free areas, ignoring the smaller ones
    */
    std::vector<FreeGoalTargetableArea> filteredFreeAreas;
    for(const auto& area : freeAreas)
    {
      if(area.interval>=GOAL_TARGET_AREA_MIN_SIZE)
      {
        filteredFreeAreas.push_back(area);
        //std::cout << ("[Robot #"+std::to_string(theRobotInfo.number)+"] FreeGoalTargetableArea: ("+std::to_string(area.begin)+","+std::to_string(area.end)+")") << "\n";
      }
    }

    /*
      2) If there is no free area, return the median point on the opponent goal line
    */
    //WATCH OUT: TEMPORARY FIX FOR THE CASE IN WHICH THERE IS NO TARGETABLE AREA, TO MANAGE IN STRIKER BEHAVIOR
    if(filteredFreeAreas.size()==0)
    {
      //std::cout<<"No free area\n";
      
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //return Vector2f(0,0);
      return Vector2f(theFieldDimensions.xPosOpponentGroundline,0);
    } 

    //project my line of view onto the goal line
    float myGazeProjection = projectGazeOntoOpponentGroundline();
    //std::cout << "myGazeProjection: "+std::to_string(myGazeProjection) << "\n";

    float targetPoint = 0;

    float minTargetableAreaDistance=INFINITY;

    //CASE 1: I'm near the goal post -> choose the nearest free area
    if(shootASAP || (std::abs(theFieldDimensions.xPosOpponentGroundline-theRobotPose.translation.x())<GOAL_TARGET_DISTANCE_THRESHOLD && filteredFreeAreas.size()!=0) || filteredFreeAreas.size()==1)
    {
      for(int i = 0; i < filteredFreeAreas.size(); i++)
      {
        FreeGoalTargetableArea currentArea = filteredFreeAreas.at(i);
        //CASE 1.1: Looking at free area directly
        if(myGazeProjection<currentArea.begin && myGazeProjection>currentArea.end)
        {
          //std::cout<<"Looking at free area ("+std::to_string(currentArea.begin)+","+std::to_string(currentArea.end)+")\n";
          if(myGazeProjection>currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
          else if(myGazeProjection<currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
          else targetPoint = myGazeProjection; 
          break;
        }
        //CASE 1.2: Looking away from free area
        else
        {
          //if freeArea is on the left
          if(currentArea.begin<myGazeProjection)
          { 
            float currentFreeAreaDistance=myGazeProjection-currentArea.begin;
            if(minTargetableAreaDistance>currentFreeAreaDistance)
            {
              minTargetableAreaDistance=currentFreeAreaDistance;
              targetPoint= currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
              //std::cout<<"FreeArea on the left: assigned targetPoint "+std::to_string(targetPoint)+"\n";
            }
          }
          //if freeArea is on the right
          else if(currentArea.end>myGazeProjection)
          {
            float currentFreeAreaDistance=currentArea.end-myGazeProjection;
            if(minTargetableAreaDistance>currentFreeAreaDistance)
            {
              minTargetableAreaDistance=currentFreeAreaDistance;
              targetPoint= currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
              //std::cout<<"FreeArea on the right: assigned targetPoint : "+std::to_string(targetPoint)+"\n";
            }
          }
        }
      }
    }
    //CASE 2: I'm far away from the goal post -> find the area with the highest utility
    else
    {
      //std::cout<< "Seeking best area based on its utility value: theRobotPose.translation.x(): "<< theRobotPose.translation.x()<<", distance from opponentGroundline: "<<std::abs(theRobotPose.translation.x()-theFieldDimensions.xPosOpponentGroundline)<<" > GOAL_DISTANCE_THRESHOLD: "<<GOAL_DISTANCE_THRESHOLD<<"\n";

      //The freeAreas vector is sorted in a decreasing order of utility values, so I select the first area
      targetPoint = filteredFreeAreas.at(0).midpoint;
      
    }
    //std::cout<<"targetPoint y coordinate: "<<targetPoint<<"\n";

    //If I'm too near to the goal line, shoot inside the goal, else shoot on the goal line
    Vector2f target;
    if (theRobotPose.translation.x() >= (theFieldDimensions.xPosOpponentGroundline - 600.f) &&
            std::abs(theRobotPose.translation.y()) < 500.f )
        target = Vector2f(theFieldDimensions.xPosOpponentGroundline + 1000.f, targetPoint);
    else
        target = Vector2f(theFieldDimensions.xPosOpponentGroundline, targetPoint);
      return target;
  }
 

Pose2f LibCheckProvider::glob2Rel(float x, float y)
{
    Vector2f result;
    float theta = 0;
    float tempX = x - theRobotPose.translation.x();
    float tempY = y - theRobotPose.translation.y();

    result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
    result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

Pose2f LibCheckProvider::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = (float)(sqrt((x * x) + (y * y)));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Pose2f(result.x(),result.y());
}
float LibCheckProvider::radiansToDegree(float x)
{
  return (float)((x*180)/3.14159265358979323846);
}
Vector2f LibCheckProvider::updateDefender()
{
    return Vector2f(theFieldDimensions.xPosOwnPenaltyArea+500, -700.f);//different from original code//.xPosOwnPenaltyMark+100, -700.f);
}

Vector2f LibCheckProvider::updateSupporter()
{
    return Vector2f(-1750.f, +700.f);
}

Vector2f LibCheckProvider::updateGoalie()
{
    Pose2f globBall = rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());

                    //-5055
    float deltaX = ((theFieldDimensions.xPosOwnGoal - globBall.translation.x()));// * 0.5f);
    float deltaY = ((globBall.translation.y()));// * 0.5f);

         if(deltaX > theFieldDimensions.xPosOwnPenaltyArea)
             deltaX = theFieldDimensions.xPosOwnPenaltyArea - 200 ;

         if (deltaY < theFieldDimensions.yPosRightGoal + 200 )
             deltaY = theFieldDimensions.yPosRightGoal + 200;

         if (deltaY > theFieldDimensions.yPosLeftGoal - 200 )
             deltaY = theFieldDimensions.yPosLeftGoal - 200;

         else if((deltaY < -200 && deltaY > theFieldDimensions.yPosRightGoal + 200) || (deltaY > 200 && deltaY < theFieldDimensions.yPosRightGoal - 200))
            deltaY = deltaY/2;


        //     std::cout << "x: " << deltaX << std::endl;
        //     std::cout << "y: " << deltaY << std::endl;
    //std::cout<< "LIBCODERELEASE" << deltaX << "  "<< deltaY<<std::endl;
     return Vector2f(deltaX, deltaY);
}

bool LibCheckProvider::isGoalieInStartingPosition() {

   if( isValueBalanced(theRobotPose.translation.x(), SPQR::GOALIE_BASE_POSITION_X+1000, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
            isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y+1000, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
        return true;
    else
        return false;
}

bool LibCheckProvider::isValueBalanced(float currentValue, float target, float bound) {
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
}

bool LibCheckProvider::isBallInKickAwayRange()
{
    if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE )
        return true;
    else
        return false;
}

bool LibCheckProvider::isGoalieInKickAwayRange()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea+100) && between(gloBall.translation.y(), theFieldDimensions.yPosRightPenaltyArea-100, theFieldDimensions.yPosLeftPenaltyArea+100))
        return true;
    else
        return false;
}

bool LibCheckProvider::isBallInArea()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea) && between(gloBall.translation.y(), theFieldDimensions.yPosRightPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea))
        return true;
    else
        return false;
}

bool LibCheckProvider::isGoalieInArea()
{
    if (between(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea) && between(theRobotPose.translation.y(), theFieldDimensions.yPosRightPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea))
        return true;
    else
        return false;
}

bool LibCheckProvider::isGoalieInAngle()
{
    //~ if(isBallInCoverRange())
        //~ if (between(angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()),
                //~ Angle::fromDegrees(-10.f),
                //~ Angle::fromDegrees(10.f) ) )
            //~ return true;
        //~ else
            //~ return false;
    //~ else
        if (between(theRobotPose.rotation, Angle::fromDegrees(-10.f), Angle::fromDegrees(10.f) ))
            return true;
        else
            return false;
}

float LibCheckProvider::angleToTarget(float x, float y)
{
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    // std::cerr << "y relativa: "<< relativePosition.translation.y() << ", x relativa: "<<relativePosition.translation.x() << std::endl;
    //return radiansToDegree(atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    //return glob2Rel(x, y).translation.angle();
}

std::vector<NodePF> LibCheckProvider::initialize_PF(float cell_size)
{
  std::vector<NodePF> potential_field;
  //init obstacle model
  int d = cell_size;
  for(int i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= d) //TODO parametrize
  {
      for(int j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= d)
      {
          potential_field.push_back( NodePF(Vector2f( i-d/2,j-d/2 ), Vector2f()) );
          if(i==SPQR::FIELD_DIMENSION_X - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
          if(i== -SPQR::FIELD_DIMENSION_X + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
          if(j== SPQR::FIELD_DIMENSION_Y - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
          if(j== -SPQR::FIELD_DIMENSION_Y + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
          if(i>=SPQR::FIELD_DIMENSION_X-(SPQR::FIELD_DIMENSION_X/50) && j<=800 && j >= -800) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
      }
  }
  return potential_field;
}

std::vector<NodePF> LibCheckProvider::compute_striker_attractive_PF(Vector2f goal, float RO, float Kap, float Kbp, float Kr, float TEAMMATE_CO, float ETA, float GAMMA)
{

  //Attractive potential field toward the goalTarget
  std::vector<NodePF> attractive_field(theStrikerPFModel.potential_field.size());

  for(unsigned int i=0; i<attractive_field.size(); ++i)
  {
    Vector2f current_cell_pos = theStrikerPFModel.potential_field.at(i).position;

    Vector2f tmp_err = goal - current_cell_pos;

    if( tmp_err.norm() <= RO)
    {
      Vector2f conic_field = Vector2f(Kap * tmp_err.x(), Kap * tmp_err.y());
      attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),conic_field);
    }
    else
    {
      Vector2f quadratic_field = Vector2f((Kbp/tmp_err.norm()) * tmp_err.x(), (Kbp/tmp_err.norm()) * tmp_err.y());
      attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),quadratic_field);
    }
  }

  return attractive_field;
}

std::vector<NodePF> LibCheckProvider::compute_striker_repulsive_PF(float RO, float Kap, float Kbp, float Kr, float TEAMMATE_CO, float ETA, float GAMMA) {
  
  Vector2f my_pos = theRobotPose.translation;

  //Build a vector out of all obstacles, including OPPONENTS and TEAMMATES, excluding POLES
  std::vector<Vector2f> repulsive_obstacles;    
  for(auto obs : theTeamPlayersModel.obstacles)
  {
    /*NOTICE: poles are added statically (a priori) by the vision system
      so the 4 goal poles will always be in the obstacle list*/
    switch(obs.type)
    {
      case Obstacle::Type::goalpost:
      {
        break;
      }
      default:
      {
        repulsive_obstacles.push_back(obs.center);
        break;
      }
    }
  }

  //Repulsive potential field from other robots and/or regions of the field, written as an opposite-attractive component
  std::vector<NodePF> repulsive_field(theStrikerPFModel.potential_field.size());

  for(unsigned int i=0; i<repulsive_field.size(); ++i)
  {
      Vector2f current_cell_pos = theStrikerPFModel.potential_field.at(i).position;
    
      repulsive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(),current_cell_pos.y()), Vector2f(0,0));

      for(unsigned int r=0; r<repulsive_obstacles.size(); ++r)
      {

          if(r == (uint) theRobotInfo.number-1) continue;
          if (repulsive_obstacles.at(r).x() == 0 || repulsive_obstacles.at(r).y() == 0) continue;

          Vector2f tmp_err = repulsive_obstacles.at(r) - current_cell_pos;
          if( (tmp_err).norm() < ETA)
          {
              float tmp_eta = (repulsive_obstacles.at(r)-my_pos).norm();

              Vector2f tmp_rep = repulsive_obstacles.at(r)-my_pos;


              repulsive_field.at(i).potential += Vector2f( -tmp_rep * (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (1/tmp_err.norm()));
              /// original else bbody
              /// repulsive_field.at(i)  += (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (-tmp_rep*(1/tmp_err.translation.absFloat()));

          }
          else
          {
              repulsive_field.at(i).potential += Vector2f(0.f, 0.f);
          }
      }
  }

  return repulsive_field;
}

std::vector<NodePF> LibCheckProvider::computePF(std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field, float cell_size)
{

  std::vector<NodePF> potential_field = initialize_PF(cell_size);

///////////////////////////////
//NOTICE: can be parallelized//
///////////////////////////////

  for(unsigned int p=0; p<potential_field.size(); ++p)
  {
    potential_field.at(p).potential = (attractive_field.at(p).potential + repulsive_field.at(p).potential);
  }

  //attractive_field.clear();
  //repulsive_field.clear();
  //potential_field.clear();

  return potential_field;
}
