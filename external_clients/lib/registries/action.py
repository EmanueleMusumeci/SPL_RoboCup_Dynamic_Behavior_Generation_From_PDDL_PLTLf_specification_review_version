from typing import Type, List, Dict, Any, Tuple, Callable

from lib.registries.values import ValueRegistry, Value
from lib.misc.singleton import Singleton
from lib.registries.registry import ConstantRegistryItem, Registry, RegistryItem, SimpleRegistryItem


class Action:
    def __init__(self, parameters : List[Tuple[str, Any]] = []):
        for parameter in parameters:
            if isinstance(parameter[1], RegistryItem):
                assert isinstance(parameter[1], Value)
            else:
#TODO: at some point, constant action arguments will have to be automatically turned into ValueRegistry items, 
#       but for now we allow native types only if they are the same ones supported by SimpleValues or FunctionalValues
                assert type(parameter[1]) in [bool, float, int, str, Callable]

        self.parameters = parameters
    
    def get_parameter_string(self):
        #param_string = "["
        param_string = ""
        for i,parameter_tuple in enumerate(self.parameters):
            param_name = parameter_tuple[0]
#TODO: When only RegistryItems will be allowed, change this
            if isinstance(parameter_tuple[1], RegistryItem):
                param_value = parameter_tuple[1].get()
            else:
                param_value = parameter_tuple[1]
            
            if isinstance(param_value, str):
                param_value_str = "'"+param_value+"'" 
            elif isinstance(param_value, RegistryItem):
                param_value_str = str(param_value.get())
            else:
                param_value_str = str(param_value)
                

            param_type = type(param_value).__name__

            param_string += param_name + ":" + param_value_str+","+ param_type
            if i < len(self.parameters) - 1:
                param_string += "/"
        #param_string += "]"

        return param_string

class StaticParameterAction(ConstantRegistryItem, Action):
    def __init__(self, name_in_registry : str, robot_skill_name : str, registry_instance : Type["ActionRegistry"], parameters : List[Tuple[str, Value]] = []):
        ConstantRegistryItem.__init__(self, name_in_registry=name_in_registry, constant_value = robot_skill_name, allowed_types = [str], registry_instance = registry_instance)
        Action.__init__(self, parameters=parameters)
        

    def get_robot_skill_name(self):
        return self.get()

    def __str__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

    def __repr__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())


class DynamicParameterAction(ConstantRegistryItem, Action):
    def __init__(self, name_in_registry : str, robot_skill_name : str, registry_instance : Type["ActionRegistry"], parameters : List[Tuple[str, Value]] = []):
        ConstantRegistryItem.__init__(self, name_in_registry=name_in_registry, constant_value = robot_skill_name, allowed_types = [str], registry_instance = registry_instance)
        Action.__init__(self, parameters=parameters)
        

    def get_robot_skill_name(self):
        return self.get()

    def __str__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

    def __repr__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

class ActionRegistry(Registry):

    ITEM_PREFIX = "action_"

    def __init__(self, robot_idle_skill : str = None, idle_skill_parameters : List = []):
        super().__init__(Action)

        self.__actions = {}
        if robot_idle_skill is not None:
            if idle_skill_parameters:
                self["action_idle"] = (robot_idle_skill, idle_skill_parameters)
            else:
                self["action_idle"] = robot_idle_skill
    
    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def set(self, action_name : str, action_data, robot_number : int = None, robot_role : str = None):
        assert isinstance(action_data, str) or isinstance(action_data, tuple)
        if isinstance(action_data, tuple):
            assert len(action_data) == 2 
            assert isinstance(action_data[0], str)
            assert isinstance(action_data[1], list)
            assert len(action_data[1]) > 0
            assert isinstance(action_data[1][0], tuple)
            assert len(action_data[1][0]) == 2
            assert isinstance(action_data[1][0], tuple)
            assert isinstance(action_data[1][0][0], str)
        
        action_name = self.get_complete_name(action_name)

        '''
        if not action_name.startswith(ActionRegistry.ITEM_PREFIX):
            action_name = ActionRegistry.ITEM_PREFIX + action_name
        '''

        if isinstance(action_data, tuple):
            self._items[action_name] = StaticParameterAction(action_name, robot_skill_name = action_data[0], registry_instance = ActionRegistry(), parameters = action_data[1])
        elif isinstance(action_data, str):
            self._items[action_name] = StaticParameterAction(action_name, robot_skill_name = action_data, registry_instance = ActionRegistry())
        else:
            raise AssertionError
        
    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def set_robot_item(self, robot_number : int, item_name : str, item):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def set_robot_role_item(self, robot_role : str, item_name : str, item):
        raise NotImplementedError




    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_robot_item(self, robot_number : int, item_name : str):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_robot_role_item(self, robot_role : str, item_name : str):
        raise NotImplementedError
            
    

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove(self, item_name : str):
        assert isinstance(item_name, str)

        item_name = self.get_complete_name(item_name)

        if item_name in self._items.keys():
            del self._items[item_name]
        else:
            raise KeyError("Unknown item: %s" % (item_name))

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove_robot_item(self, robot_number : int, item_name : str):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove_robot_role_item(self, robot_role : str, item_name : str):
        raise NotImplementedError

    

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_complete_name(self, action_name : str, robot_number : int = None, robot_role : str = None):
        action_name = action_name.lower()
        if not action_name.startswith(ActionRegistry.ITEM_PREFIX):
            action_name = ActionRegistry.ITEM_PREFIX + action_name
        return action_name