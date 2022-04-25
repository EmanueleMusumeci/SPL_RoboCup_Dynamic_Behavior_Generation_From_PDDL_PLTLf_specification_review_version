
import os,inspect
from typing import Dict, List

from lib.experiment import setup_conditioned_FOND_policy_for_experiment, ExperimentType, check_adjacency_ready_problem
from lib.benchmarks import generate_adjacency_reticle_string

def get_experiment_type():
    return ExperimentType.POLICY

def get_base_problem_name():
    return "striker_with_battery_conditioning"

def get_robot_formation():
    return {3 : "Caligola"}

def base_role_to_benchmark_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)

    return {
        "striker" : {
            "goal" : "isat_ball_goaltarget",
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_with_battery_conditioning_domain.pddl"),
            "pddl_base_problem_path" : os.path.join(currentdir, "PDDL", "striker_with_battery_conditioning_problem.pddl"),
            "pddl_generated_problems_dir" : None,
            #"pddl_mapping_path" : os.path.join(currentdir, "PDDL", "simple_striker_mapping.map"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),

            "generation_parameters" : {
                "min_x" : 3,
                "max_x" : 20,
                "min_y" : 3,
                "max_y" : 3,
                "allow_diagonal_adjacency" : True
            },
            "constrainable_predicates" : ["isat", "highbatteryconsumption"],
        }
    }

def generate_problems(role, generated_problems_dir, adjacency_predicates_placeholder_token = "ADJACENCY_PREDICATES", pddl_adjacency_predicate = "adjacent"):
    assert role in generation_data.keys()
    
    generation_data = base_role_to_benchmark_generation_data()
    base_problem_path = generation_data["pddl_base_problem_path"]

    role_generated_problems_dir = os.path.join(generated_problems_dir, role)
    if not os.path.exists(role_generated_problems_dir):
        os.makedirs(role_generated_problems_dir)
    
    base_problem_name = get_base_problem_name()

    with open(base_problem_path, mode="r") as base_problem_file:
        #Check that the file is adjacency ready
        assert check_adjacency_ready_problem(base_problem_path)

        generation_parameters = generation_data["generation_parameters"]
        for waypoints_x in range(generation_parameters["min_x"], generation_parameters["max_x"]):
            for waypoints_y in range(generation_parameters["min_y"], generation_parameters["max_y"]):
                generated_problem_name = base_problem_name + "__min_x_" + str(waypoints_x) + "_min_y_" + str(waypoints_y) + ".pddl"
                generated_problem_path = os.path.join(role_generated_problems_dir, generated_problem_name)
                generated_problem_text = ""
                for line in base_problem_file.readlines():
                    if "ADJACENCY_PREDICATES" in line:
                        line = line.replace("ADJACENCY_PREDICATES", generate_adjacency_reticle_string(pddl_adjacency_predicate, waypoints_x, waypoints_y, allow_diagonal_adjacency=generation_data["allow_diagonal_adjacency"]))
                    generated_problem_text += line

                with open(generated_problem_path, mode="w+") as generated_problem_file:
                        generated_problem_file.writelines(generated_problem_text)

def perform_benchmark(role_to_additional_constraints : Dict[str, List[str]] = {}):
    return setup_conditioned_FOND_policy_for_experiment(get_base_problem_name(), base_role_to_benchmark_generation_data(), role_to_additional_constraints = role_to_additional_constraints)

def initialize_registries():
    #sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance


    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("moverobot", "ReachPosition", [2])
    ActionRegistry().register_action_template("kickball", "Kick", [3])
    ActionRegistry().register_action_template("carryball" ,"CarryBall", [3])
    ActionRegistry().register_action_template("kicktogoal", "CarryAndKickToGoal", [])

    #Ground :objects in ValueRegistry
    ValueRegistry()["kickingposition"] = (2000, 0)
    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="strikercurrentposition")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ballcurrentposition")
