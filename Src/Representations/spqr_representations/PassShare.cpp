/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "PassShare.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"

#include <iostream>


//#define PLOT_SINGE_TSL(name) \
//  PLOT("representation:FieldFeatureOverview:TimeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));
//

void PassShare::operator >> (BHumanMessage& m) const
{
  m.theBHumanArbitraryMessage.queue.out.bin << passTarget.translation.x();
  m.theBHumanArbitraryMessage.queue.out.bin << passTarget.translation.y();
  m.theBHumanArbitraryMessage.queue.out.bin << myNumber;
  m.theBHumanArbitraryMessage.queue.out.bin << passingTo;
  m.theBHumanArbitraryMessage.queue.out.bin << readyReceive;
  m.theBHumanArbitraryMessage.queue.out.bin << readyPass;

  // OLD PASSSHARE REPRESENTATION
  // m.theBHumanArbitraryMessage.queue.out.bin << sharedGoalUtil;
  // m.theBHumanArbitraryMessage.queue.out.bin << myNumber;
  // m.theBHumanArbitraryMessage.queue.out.bin << passingTo;
  // m.theBHumanArbitraryMessage.queue.out.bin << readyReceive;
  // m.theBHumanArbitraryMessage.queue.out.bin << readyPass;
  // m.theBHumanArbitraryMessage.queue.out.bin << role;

  //m.theBSPLStandardMessage.role = (float)role;
  //std::cout<<"role ="<<role<<std::endl;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

bool PassShare::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> passTarget.translation.x();
  m.bin >> passTarget.translation.y();
  m.bin >> myNumber;
  m.bin >> passingTo;
  m.bin >> readyReceive;
  m.bin >> readyPass;

  // OLD PASSSHARE REPRESENTATION
  // ASSERT(m.getMessageID() == id());

  // m.bin >> sharedGoalUtil;

  // m.bin >> myNumber;
  // m.bin >> passingTo;
  // m.bin >> readyReceive;
  // m.bin >> readyPass;
  // m.bin >> role;

  return true;
}

/*
void PassShare::draw() const
{
  #ifdef TARGET_SIM
  // drawing of the model in the field view
        
  DEBUG_DRAWING3D("representation:PassShareDebug", "field")
  {
    if(theRole.role==Role::RoleType::striker || theRole.role==Role::RoleType::undefined || OPPONENT_GOAL_MODEL_DEBUG_MODE)
    {
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const PassShare& thePassShare = static_cast<const PassShare&>(Blackboard::getInstance()["PassShare"]);

      LINE3D("representation:PassShareDebug", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, thePassShare.passTarget.translation.x(), thePassShare.passTarget.translation.y(), 10, 1, ColorRGBA(255,0,0));
    }
  }
  #endif
}*/
