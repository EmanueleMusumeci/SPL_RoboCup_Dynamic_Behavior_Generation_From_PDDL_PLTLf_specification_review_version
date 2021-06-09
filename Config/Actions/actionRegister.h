#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Communication/BHumanMessage.h"

namespace Actions
{
    ENUM(ActionType,
    {,
        Idle,
        ReachPosition,
        ReachBall,
        CarryBall,
        Kick,
        CarryAndKickToGoal,
        //LookAt,
        //SaySomething,
    });
}