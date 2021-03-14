/**
 * @file WalkToCarry.cpp
 *
 * This file implements the implementation of the WalkToCarry skill. Similar to the WalkToCarry skill
 * but specific for the ball carrier card
 * 
 * @author Emanuele Musumeci (based on Emanuele Antonioni's WalkToCarry)
 */
 
  // *******      NOTE: THIS FILE HAS BEEN MERGED WITH WalkToPass.cpp

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

SKILL_IMPLEMENTATION(WalkToCarryImpl,
{,
  IMPLEMENTS(WalkToCarry),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(PassShare),
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

class WalkToCarryImpl : public WalkToCarryImplBase
{
  void execute(const WalkToCarry& p) override
  {
    Angle target_angle = ( theRobotPose.inversePose * Vector2f(p.target.translation.x(), p.target.translation.y() ) ).angle();
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = Pose2f( target_angle, theFieldBall.positionRelative.x() - p.offsetX,
                                                theFieldBall.positionRelative.y() - p.offsetY );
    theMotionRequest.walkRequest.speed = Pose2f(1.f,1.f,1.f);
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToCarry&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToCarryImpl);
