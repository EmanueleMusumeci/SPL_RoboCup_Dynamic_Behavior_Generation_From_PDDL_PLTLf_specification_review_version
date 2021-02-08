/**
 * @file GoalTarget.cpp
 *
 * This file implements the implementation of the GoalTarget skill.
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"

SKILL_IMPLEMENTATION(GoalTargetImpl,
{,
  IMPLEMENTS(GoalTarget),
  REQUIRES(LibCheck),
  MODIFIES(BehaviorStatus),
});

class GoalTargetImpl : public GoalTargetImplBase
{
  void execute(const GoalTarget& p) override
  {
    theBehaviorStatus.shootingTo = p.goalTarget;
  }
};

MAKE_SKILL_IMPLEMENTATION(GoalTargetImpl);
