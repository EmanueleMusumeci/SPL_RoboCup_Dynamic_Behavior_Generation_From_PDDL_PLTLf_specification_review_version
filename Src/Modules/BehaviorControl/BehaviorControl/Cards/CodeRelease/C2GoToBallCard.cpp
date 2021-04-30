/**
 * @file CodeReleasePositionForKickOffCard.cpp
 *
 * This file implements nothing.
 *
 * @author Tommaso Carlini & Graziano Specchi
 */

// TODO: Scrivere il behaviour

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(C2GoToBallCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  REQUIRES(GameInfo),
});

class C2GoToBallCard : public C2GoToBallCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);
    theLookForwardSkill();
    theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("Please implement a behavior for me!");
  }
};

MAKE_CARD(C2GoToBallCard);

