
from ast import alias
import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler

import fond4ltlfpltlf
from fond4ltlfpltlf.core import execute

def get_robot_formation():
    return {3 : "Caligola"}

def setup_experiment():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)
    sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance

    #from robocup_spl_temporal_goals.third_party.planning_with_past.planners.downward import DownwardPlanner



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

    domain_path = os.path.join(currentdir, "PDDL", "acrobatics_domain.pddl")
    problem_path = os.path.join(currentdir, "PDDL", "acrobatics_problem.pddl")
    striker_mapping_path = os.path.join(currentdir, "PDDL", "striker_mapping_fond.map")
    working_dir = os.path.join(currentdir, "output")

    simple_approacher_pltlf_goal = "F(up & position_p1)"
    print("Creating Policy for striker role from domain file: %s with problem file %s and PLTLf formula '%s'" % (domain_path, problem_path, simple_approacher_pltlf_goal))
    try:
        simple_approacher_plan = Policy.build_from_FOND_PDDL_and_PLTLf_formula_with_fond4ltlfpltlf(domain_path, problem_path, working_dir, simple_approacher_pltlf_goal, mapping_path = striker_mapping_path)
        simple_approacher_plan.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "plan_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
        plan_handler = PolicyHandler(simple_approacher_plan, plan_postprocessing_functions = [])
    except AssertionError as e:
        raise e
    else:
        print("OK")
    
    return {"striker" : plan_handler}