/**
 * @file PassShare.h
 * PassShare representation
 */

#pragma once


#include "Tools/Math/Pose2f.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

STREAMABLE(PassShare, COMMA public PureBHumanArbitraryMessageParticle<idPassShare>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  
  float x,
  (Pose2f) passTarget,
  (int) myNumber,
  (int) readyReceive,
  (int) readyPass,

  // OLD PASS SHARE REPRESENTATION
  /* float x, */

  (float) sharedGoalUtil,
  /* (int) myNumber, */
  (int) (0) passingTo,
  /* (int) readyReceive, */
  /* (int) readyPass,  */
  (int) role,

});
/*
STREAMABLE(PassShareDebug,
{
  void draw() const,
});*/