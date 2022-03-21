import os
import sys

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )

from lib.registries.action import ActionRegistry
from lib.registries.values import ValueRegistry

if __name__ == "__main__":
    print("Tests: ActionRegistry creation and print")
    action_registry = ActionRegistry(robot_idle_skill="Idle")
    action_registry["action_kick_ball"] = "kick"
    ActionRegistry()["action_reach_ball"] = "reachBall"
    print(ActionRegistry())
    print("\n\n")

    print("Tests: ValueRegistry creation and print")
    value_registry = ValueRegistry()
    ValueRegistry()["threshold1"] = 200
    ValueRegistry()["threshold2"] = 50
    print(ValueRegistry())
    print("\n\n")