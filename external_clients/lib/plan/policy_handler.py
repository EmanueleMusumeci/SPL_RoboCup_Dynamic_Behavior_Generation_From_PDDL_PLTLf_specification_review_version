import time

from lib.plan.policy import Policy, PolicyEdge, PolicyNode 


from lib.registries.action import *

#TODO: get_current_state should 1) Update the Policy 2) return the literals and are actions for the current state (only action is needed but we want this to be more general purpose)
class PolicyHandler:
    def __init__(self, policy : Policy, plan_postprocessing_functions = []):
        assert isinstance(policy, Policy), "This instance should wrap a networkx DiGraph"
        self.policy = policy
        if plan_postprocessing_functions:
            for preprocessing_step in plan_postprocessing_functions:
                preprocessing_step(policy)
        self.__current_state : PolicyNode = policy.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_instance("action_idle"), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    
    def get_current_action(self):
        self.update()
        #print(self.__trace)
        return self.__trace[-1]["performed_action"]

    def get_previous_action(self):
        if len(self.__trace) == 1:
            return ActionRegistry()["action_idle"]
        else:
            return self.__trace[-2]["performed_action"]
    
    def get_current_state(self):
        self.update()
        return self.__current_state.node_id

    def get_previous_state(self):
        return self.__previous_state.node_id

    def get_last_transition_timestamp(self):
        return self.__trace[-1]["timestamp"]
    
    def update(self):
        outgoing_edges : List[PolicyEdge] = self.__current_state.get_outgoing_edges()

        #Select only edges where all literals are verified
        verified_edge = None
        #print(outgoing_edges)
        if self.policy.is_plan:
            assert len(outgoing_edges) == 1 or len(outgoing_edges) == 0
        
        for edge in outgoing_edges:
            if edge.is_verified():
                assert verified_edge is None, "More than one verified edge available"
                    
                verified_edge = edge
                #Even if we found a verified edge, keep looping to check consistency of this plan/policy
                # (a plan/policy always has ONE verified outgoing edge at each time)
                

        print("\n"+str(verified_edge))
        
        
        print(self.__trace[-1]["performed_action"].completed)
        if self.__trace[-1]["performed_action"].completed or self.__trace[-1]["performed_action"] == ActionRegistry().get_instance("action_idle"):
            #print("CHOOSING BEST ACTION")
            chosen_transition : PolicyEdge = verified_edge
        else:
            print("USING PREVIOUS ACTION")
            chosen_transition = self.__trace[-1]["edge"]
        
        #print(chosen_transition)
        if chosen_transition is not None:
            chosen_action : Action = chosen_transition.guard_action
            destination_state : PolicyNode = chosen_transition.to_node
        else:
            chosen_action : Action = ActionRegistry().get_instance("action_idle")
            destination_state : PolicyNode = self.__current_state

        #If we're not still repeating the same current action
        if chosen_transition != self.__current_edge:
            print("UPDATING TRACE WITH CHOSEN ACTION '%s'" % (str(chosen_action)))
            #Update previous state/edge and trace in case the new one is different from the previous one
            self.__previous_edge = self.__current_edge
            self.__previous_state = self.__current_state

            self.__trace.append({"edge" : chosen_transition, "performed_action" : chosen_action, "destination_state" : destination_state, "timestamp" : time.time()})

            print(self.__trace)

            #Transition through the chosen edge
            self.__current_state = destination_state
            self.__current_edge = chosen_transition

#TODO: add some logic to choose an edge (maybe planning techniques)    
    def choose_best_edge(self, edges):
        return edges[0]

    def __str__(self):
        return "PolicyHandler:\nCurrent state: %s\n,Policy: %s" % (str(self.__current_state), str(self.dfa)) 

