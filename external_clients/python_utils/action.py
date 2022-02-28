from typing import Type
from bool import ValueRegistry

from singleton import Singleton


class Action:
    def __init__(self, action_name_in_registry : str, robot_skill_name : str):
        assert isinstance(action_name_in_registry, str)
        assert isinstance(robot_skill_name, str)
        self.name = action_name_in_registry
        self.robot_skill_name = robot_skill_name
    
    def __str__(self):
        return self.name+"('%s')" % self.robot_skill_name

class ActionRegistry(metaclass=Singleton):

    ACTION_PREFIX = "action_"

    def __init__(self):
        self.__actions = {}
        
    def __setitem__(self, action_name : str, robot_skill_name):

        if not action_name.startswith(ActionRegistry.ACTION_PREFIX):
            action_name = ActionRegistry.ACTION_PREFIX + action_name

        self.__actions[action_name] = Action(action_name, robot_skill_name)

    def __getitem__(self, key):
        return self.__actions[key]

    def __contains__(self, action_name):
        assert isinstance(action_name, str)
        action_name = self.get_complete_name(action_name)
        return action_name in self.__actions.keys()

    def __str__(self):
        return str(self.__actions)

    def __repr__(self):
        return str(self)
    
    def get_action_instance(self, action_name : str):
        assert isinstance(action_name, str)

        action_name = self.get_complete_name(action_name)

        if action_name in self.__actions.keys():
            return self.__actions[action_name]
        else:
            raise KeyError("Unknown action: %s" % (action_name))

    def get_complete_name(self, action_name : str):
        action_name = action_name.lower()
        if not action_name.startswith(ActionRegistry.ACTION_PREFIX):
            action_name = ActionRegistry.ACTION_PREFIX + action_name
        return action_name

if __name__ == "__main__":
    print("Tests: ActionRegistry creation and print")
    action_registry = ActionRegistry()
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