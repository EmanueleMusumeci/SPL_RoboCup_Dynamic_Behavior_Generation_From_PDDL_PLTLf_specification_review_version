import math 

import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler
from lib.registries.fluents import FluentRegistry

def get_robot_formation():
    return {3 : "Caligola", 2 : "Nerone"}

def setup_experiment():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)
    sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance_in_degrees

    #from planning_for_past_temporal_goals.planning_with_past.planners.downward import DownwardPlanner

    striker_domain_path = os.path.join(currentdir, "PDDL", "robocup_striker_domain_fond.pddl")
    jolly_domain_path = os.path.join(currentdir, "PDDL", "robocup_jolly_domain_fond.pddl")

    striker_problem_path = os.path.join(currentdir, "PDDL", "striker_problem_fond.pddl")
    jolly_problem_path = os.path.join(currentdir, "PDDL", "jolly_problem_fond.pddl")

    working_dir = os.path.join(currentdir, "output")


    ActionRegistry(robot_idle_skill="Idle")

    #Ground :objects in ValueRegistry
    ValueRegistry()["kicking_position"] = (2000, 0)
    ValueRegistry()["goal_position"] = (3000, 0)
    ValueRegistry()["field_sideline"] = 3000

    def jolly_receiving_position(striker_obstacles, jolly_position):
        print(striker_obstacles)
        if striker_obstacles:
            centroid = striker_obstacles[0]
            for obstacle in striker_obstacles[1:]:
                centroid = ((centroid[0] + obstacle[0])/2, (centroid[1] + obstacle[1])/2)
        else:
            centroid = jolly_position
        print(centroid)

        if centroid[1] > 0:
            return (math.radians(120), 2000, -2000)
        else:
            return (math.radians(-120), 2000, 2000)

                
    ValueRegistry().add_function(jolly_receiving_position)


    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_alias(item_name="jolly_position", alias_name="jolly-current-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-current-position")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("move-with-ball-to-kicking-position", "CarryBall", ["kicking_position"])
    ActionRegistry().register_action_template("move-to-ball", "ReachBall", [])
    ActionRegistry().register_action_template("pass-ball-to-jolly" ,"Kick", ["jolly_position"])
    ActionRegistry().register_action_template("kick-to-goal", "Kick", ["goal_position"])
    ActionRegistry().register_action_template("dribble-opponent", "CarryBall", ["goal_position"])
    ActionRegistry().register_action_template("wait-for-jolly", "Idle", [])


    def is_obstacle_blocking(position, obstacle_position, target_position):
        #print(position)
        #print(obstacle_position)
        #print(target_position)
        return obstacle_position[0] > position[0] and obstacle_position[0] < target_position[0]

    def is_obstacle_left(obstacle_position, field_sideline):
        return obstacle_position[1] > 0 and obstacle_position[1] < field_sideline
        
    def is_obstacle_right(obstacle_position,field_sideline):
        return obstacle_position[1] < 0 and obstacle_position[1] > -field_sideline

    def is_striker_obstacle_blocking_goal(striker_position, striker_obstacles, field_sideline):
        for obstacle_position in striker_obstacles:
            if is_obstacle_blocking(striker_position, obstacle_position, target_position=(4500,0)):
                return True
        return False

    FluentRegistry().add_function(is_striker_obstacle_blocking_goal, aliases=["fluent-is-striker-obstacle-blocking-goal"])

    def is_jolly_available(striker_position, jolly_position):
        return jolly_position[0] > striker_position[0]

    def is_jolly_in_position(jolly_position, jolly_receiving_position):
        return linear_distance(jolly_position, jolly_receiving_position) < 500

    def is_jolly_aligned_to_striker(jolly_position, jolly_receiving_position):
        return angular_distance_in_degrees(jolly_position, jolly_receiving_position) < 10


    FluentRegistry().add_function(is_jolly_available, aliases=["fluent-is-jolly-available"], default_value_if_not_evaluable=False)
    FluentRegistry().add_function(is_jolly_in_position, aliases=["fluent-is-jolly-in-position"], default_value_if_not_evaluable = False)
    FluentRegistry().add_function(is_jolly_aligned_to_striker, aliases=["fluent-is-jolly-aligned-to-striker"], default_value_if_not_evaluable = False)

    print("Creating Policy for striker role from domain file: %s with problem file %s" % (striker_domain_path, striker_problem_path))
    try:
        non_deterministic_striker_policy = Policy.build_from_FOND_PDDL(striker_domain_path, striker_problem_path, working_dir)
        non_deterministic_striker_policy.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy_preview", Path(os.path.abspath(__file__)).stem+"(striker).png"), show_plot = False)
        striker_policy_handler = PolicyHandler(non_deterministic_striker_policy, plan_postprocessing_functions = [])
    except AssertionError as e:
        raise e
    else:
        print("\n----------------\nOK striker policy\n----------------\n\n\n\n")






    ActionRegistry().register_action_template("move-to-receiving-position", "ReachPositionAndAngle", ["jolly_receiving_position"])
    ActionRegistry().register_action_template("turn-to-striker", "ReachPositionAndAngle", ["jolly_receiving_position"])
    #ActionRegistry().register_action_template("check-obstacle-position", "Idle", [])

    #def is_striker_obstacle_left(striker_position, striker_obstacles, field_groundline, field_sideline):
    #    for obstacle_position in striker_obstacles:
    #        if is_obstacle_blocking(striker_position, obstacle_position, target_position=(4500,0), field_groundline=field_groundline) and is_obstacle_left(obstacle_position, field_sideline):
    #            return True
    #    return False

    #def is_striker_obstacle_right(striker_position, striker_obstacles, field_groundline, field_sideline):
    #    for obstacle_position in striker_obstacles:
    #        if is_obstacle_blocking(striker_position, obstacle_position, target_position=(4500,0), field_groundline=field_groundline) and is_obstacle_right(obstacle_position, field_sideline):
    #            return True
    #    return False

    #FluentRegistry().add_function(is_striker_obstacle_left, aliases=["fluent-is-striker-obstacle-left"])
    #FluentRegistry().add_function(is_striker_obstacle_right, aliases=["fluent-is-striker-obstacle-right"])

    print("Creating Policy for jolly role from domain file: %s with problem file %s" % (jolly_domain_path, jolly_problem_path))
    try:
        non_deterministic_jolly_policy = Policy.build_from_FOND_PDDL(jolly_domain_path, jolly_problem_path, working_dir)
        non_deterministic_jolly_policy.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy_preview", Path(os.path.abspath(__file__)).stem+"(jolly).png"), show_plot = False)
        jolly_policy_handler = PolicyHandler(non_deterministic_jolly_policy, plan_postprocessing_functions = [])
    except AssertionError as e:
        raise e
    else:
        print("\n----------------\nOK jolly policy\n----------------\n\n\n\n")
    



    return {"striker" : striker_policy_handler, "jolly" : jolly_policy_handler}
    #return {"striker" : striker_policy_handler}
    #return {"striker" : PolicyHandler(Policy.Idle()), "jolly" : jolly_policy_handler}