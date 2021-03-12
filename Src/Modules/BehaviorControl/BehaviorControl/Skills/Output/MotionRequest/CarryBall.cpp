/**
 * @file CarryBall.cpp
 *
 * This skill allows a robot to bring the ball forward (used by the ball carrier)
 * 
 * @author Emanuele Musumeci
 */
 
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include <iostream>

SKILL_IMPLEMENTATION(CarryBallImpl,
{,
  IMPLEMENTS(CarryBall),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),

  CALLS(Activity),

  MODIFIES(MotionRequest),
  MODIFIES(BehaviorStatus),

  DEFINES_PARAMETERS(
  {,
    (Angle)(10_deg) angleToShootThreshold,
    
  }),
});

class CarryBallImpl : public CarryBallImplBase
{
  void execute(const CarryBall& p) override
  {
    Vector2f target = p.target.translation;

    theBehaviorStatus.shootingTo = target;

    Angle target_angle = ( theRobotPose.inversePose * Vector2f(target.x(), target.y() ) ).angle();
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = Pose2f( target_angle, theFieldBall.positionRelative.x() - p.offsetX,
                                                theFieldBall.positionRelative.y() - p.offsetY );
    theMotionRequest.walkRequest.speed = Pose2f(1.f,1.f,1.f);
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const CarryBall&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(CarryBallImpl);
