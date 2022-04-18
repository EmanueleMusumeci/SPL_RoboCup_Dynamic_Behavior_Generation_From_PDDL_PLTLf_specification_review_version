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
    
    def reset(self):
        self.__current_state : PolicyNode = self.policy.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_instance("action_idle"), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    
    def get_current_action(self):
        action = self.update()
        #print(self.__trace)
        print("Chosen action: "+str(action)+"\n------------\n")
        return action

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
        
        print("------------\nUPDATE\n------------\nCurrent node: %s" % (str(self.__current_state.node_id)))
        #Select only edges where all literals are verified
        verified_edge = None
        #print(outgoing_edges)
        if self.policy.is_plan:
            assert len(outgoing_edges) == 1 or len(outgoing_edges) == 0
        
        verified_edges = []
        for edge in outgoing_edges:
            if edge.is_verified():
                verified_edges.append(edge)
                #Even if we found a verified edge, keep looping to check consistency of this plan/policy
                # (a plan/policy always has ONE verified outgoing edge or ONE verified edge and ONE edge without conditions at each time)
        
        #print("\n"+str(verified_edges))
        
        
        if self.__trace[-1]["performed_action"].completed or self.__trace[-1]["performed_action"].is_idle_action():
            if len(verified_edges) > 1:
                assert len(verified_edges) == 2, "More than 2 edges are verified"
                assert (verified_edges[0].get_fluents() and not verified_edges[1].get_fluents()) or (not verified_edges[0].get_fluents() and verified_edges[1].get_fluents())
                if not verified_edges[0].get_fluents():
                    chosen_transition : PolicyEdge = verified_edges[1]
                else:
                    chosen_transition : PolicyEdge = verified_edges[0]

            elif not verified_edges:
                chosen_transition : PolicyEdge = None
            else:
                chosen_transition : PolicyEdge = verified_edges[0]
            print("Last action completed")
        else:
            chosen_transition = self.__trace[-1]["edge"]
            if chosen_transition is not None:
                print("Last action NOT completed\nChosing last action")
                
        #print(chosen_transition)
        if chosen_transition is not None:
            chosen_action : Action = chosen_transition.guard_action
            destination_state : PolicyNode = chosen_transition.to_node
            #print("\tCurrent action: "+str(chosen_transition.guard_action))
        else:
            chosen_action : Action = ActionRegistry().get_instance("action_idle")
            destination_state : PolicyNode = self.__current_state
            #print("\tCurrent action: Idle Action")

        #If we're not still repeating the same current action
        if chosen_transition != self.__current_edge:
            print("\tUpdating trace with chosen action '%s'" % (str(chosen_action)))
            #Update previous state/edge and trace in case the new one is different from the previous one
            self.__previous_edge = self.__current_edge
            self.__previous_state = self.__current_state

            self.__trace.append({"edge" : chosen_transition, "performed_action" : chosen_action, "destination_state" : destination_state, "timestamp" : time.time()})

            print("\t\tCurrent trace:\n"+self.trace_to_string())

            #Transition through the chosen edge
            self.__current_state = destination_state
            self.__current_edge = chosen_transition
    
        print("------------")
        return chosen_action
        
#TODO: add some logic to choose an edge (maybe planning techniques)    
    def choose_best_edge(self, edges):
        return edges[0]


    def trace_to_string(self):
        trace_string = ""
        for edge in self.__trace:
            trace_string += "\t"+str(edge["performed_action"])+"\n"
        
        return trace_string

    def __str__(self):
        return "PolicyHandler:\nCurrent state: %s\n,Policy: %s" % (str(self.__current_state), str(self.dfa)) 

