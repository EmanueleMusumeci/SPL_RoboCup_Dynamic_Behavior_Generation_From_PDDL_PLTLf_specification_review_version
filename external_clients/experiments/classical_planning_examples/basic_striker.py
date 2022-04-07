
from ast import alias
import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler


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

    domain_path = os.path.join(currentdir, "PDDL", "robocup_domain_deterministic.pddl")
    problem_path = os.path.join(currentdir, "PDDL", "simple_striker_problem_deterministic.pddl")
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
    ActionRegistry().register_action_template("kick-to-goal", "CarryAndKickToGoal", [])

    #Ground :objects in ValueRegistry
    ValueRegistry()["kicking-position"] = (2000, 0)
    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-current-position")

    print("Creating Plan from domain file: %s with problem file %s" % (domain_path, problem_path))
    try:
        simple_approacher_plan = Policy.build_from_PDDL(domain_path, problem_path, working_dir)
        simple_approacher_plan.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "plan_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
        plan_handler = PolicyHandler(simple_approacher_plan, plan_postprocessing_functions = [])
    except AssertionError as e:
        raise e
    else:
        print("OK")
    
    return {"striker" : plan_handler}