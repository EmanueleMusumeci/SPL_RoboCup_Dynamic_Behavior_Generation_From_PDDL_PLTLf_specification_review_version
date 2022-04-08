
from ast import alias
import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler
from lib.registries.fluents import FluentRegistry


def setup_experiment():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)
    sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import distance

    #from planning_for_past_temporal_goals.planning_with_past.planners.downward import DownwardPlanner

    striker_domain_path = os.path.join(currentdir, "PDDL", "robocup_striker_domain_fond.pddl")
    supporter_domain_path = os.path.join(currentdir, "PDDL", "robocup_supporter_domain_fond.pddl")
    striker_problem_path = os.path.join(currentdir, "PDDL", "striker_problem_fond.pddl")
    supporter_problem_path = os.path.join(currentdir, "PDDL", "supporter_problem_fond.pddl")

    working_dir = os.path.join(currentdir, "output")


    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("move-robot", "ReachPosition", [2])
    ActionRegistry().register_action_template("kick-ball", "Kick", [3])
    ActionRegistry().register_action_template("carry-ball" ,"CarryBall", [3])
    ActionRegistry().register_action_template("pass-ball-to-supporter" ,"Kick", [])
    ActionRegistry().register_action_template("kick-to-goal", "CarryAndKickToGoal", [])

    #Ground :objects in ValueRegistry
    ValueRegistry()["kicking-position"] = (2000, 0)
    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_alias(item_name="supporter_position", alias_name="supporter-current-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-current-position")

#TODO: improve
    def is_obstacle_blocking(position, obstacle_position, target_position):
        return obstacle_position[0] > position[0]

    def is_opponent_blocking_goal(striker_position, striker_obstacles):
        is_obstacle_blocking_striker = False
        for obstacle_position in striker_obstacles:
            is_obstacle_blocking_striker = is_obstacle_blocking_striker or is_obstacle_blocking(striker_position, obstacle_position, target_position=(3000,0))
        return is_obstacle_blocking_striker

    FluentRegistry().add_function(is_opponent_blocking_goal)
    FluentRegistry().register_alias(item_name="is_opponent_blocking_goal", alias_name="is-opponent-blocking-goal")

    print("Creating Policy for striker role from domain file: %s with problem file %s" % (striker_domain_path, striker_problem_path))
    try:
        non_deterministic_striker_policy = Policy.build_from_FOND_PDDL(striker_domain_path, striker_problem_path, working_dir)
        non_deterministic_striker_policy.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy_preview", Path(os.path.abspath(__file__)).stem+"(striker).png"), show_plot = False)
        striker_policy_handler = PolicyHandler(non_deterministic_striker_policy, plan_postprocessing_functions = [])
    except AssertionError as e:
        raise e
    else:
        print("OK striker policy")

    #print("Creating Policy for supporter role from domain file: %s with problem file %s" % (supporter_domain_path, supporter_problem_path))
    #try:
    #    non_deterministic_supporter_policy = Policy.build_from_FOND_PDDL(supporter_domain_path, supporter_problem_path, working_dir)
    #    non_deterministic_supporter_policy.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy_preview", Path(os.path.abspath(__file__)).stem+"(supporter).png"), show_plot = False)
    #    supporter_policy_handler = PolicyHandler(non_deterministic_supporter_policy, plan_postprocessing_functions = [])
    #except AssertionError as e:
    #    raise e
    #else:
    #    print("OK supporter policy")
    
    raise
    return {"striker" : striker_policy_handler, "supporter" : supporter_policy_handler}