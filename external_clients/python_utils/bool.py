from turtle import distance
from typing import Callable, List, Type
from abc import abstractmethod
import inspect

from numpy import isin

from singleton import Singleton

class ValueRegistry(metaclass=Singleton):

    def __init__(self):
        self.__values = {}
    
    def set_robot_value(self, robot_number : int, value_name : str, value):
        self.__values[str(robot_number)+"_"+value_name] = value

    def get_robot_value(self, robot_number : int, value_name : str):
        return self.__values[str(robot_number)+"_"+value_name]
    
    def __setitem__(self, key, value):
        self.__values[key] = value

    def __getitem__(self, key):
        return self.__values[key]

    def __contains__(self, key):
        return key in self.__values.keys()

    def __str__(self):
        return str(self.__values)

    def __repr__(self):
        return str(self)


class LiteralRegistry(metaclass=Singleton):

    LITERAL_PREFIX = "literal_"

    def __init__(self):
        self.__literals = {}
    
    def __setitem__(self, literal_name : str, literal):
        assert isinstance(literal_name, str)
        assert isinstance(literal, SimpleLiteral) or isinstance(literal, FunctionLiteral) or isinstance(literal, bool) or isinstance(literal, Callable)
        
        literal_name = self.get_complete_name(literal_name)

        if isinstance(literal, SimpleLiteral) or isinstance(literal, FunctionLiteral):
            self.__literals[literal_name] = literal
        elif isinstance(literal, bool):
            if literal_name in self.__literals.keys():
                self.__literals[literal_name].set(literal)
            else:
                self.__literals[literal_name] = SimpleLiteral(literal_name, value=literal)
        elif isinstance(literal, Callable):
            if literal_name in self.__literals.keys():
                self.__literals[literal_name].set(literal)
            else:
                self.__literals[literal_name] = FunctionLiteral(literal_name, formula=literal)
        else:
            raise Exception("Should not be getting here")

    def __getitem__(self, literal_name : str):
        assert isinstance(literal_name, str)

        literal_name = self.get_complete_name(literal_name)

        if literal_name in self.__literals.keys():
            return bool(self.__literals[literal_name])
        else:
            raise KeyError("Unknown literal: %s" % (literal_name))
    
    def is_function_literal(self, literal_name : str):

        literal_name = self.get_complete_name(literal_name)

        assert isinstance(literal_name, str)
        if literal_name in self.__literals.keys():
            return isinstance(self.__literals[literal_name], FunctionLiteral)
        else:
            raise KeyError("Unknown literal: %s" % (literal_name))
    
    def add_function_literal(self, function : Callable):
        literal_name = self.get_complete_name(function.__name__)

        self[literal_name] = FunctionLiteral(literal_name, formula=function)
        
    def get_literal_instance(self, literal_name : str):
        assert isinstance(literal_name, str)

        literal_name = self.get_complete_name(literal_name)

        if literal_name in self.__literals.keys():
            return self.__literals[literal_name]
        else:
            raise KeyError("Unknown literal: %s" % (literal_name))
    
    def __contains__(self, literal_name : str):
        assert isinstance(literal_name, str)
        literal_name = self.get_complete_name(literal_name)
        return literal_name in self.__literals.keys()
    
    def __str__(self):
        return str(self.__literals)

    def __repr__(self):
        return str(self)
    
    def get_complete_name(self, literal_name : str):
        literal_name = literal_name.lower()
        if not literal_name.startswith(LiteralRegistry.LITERAL_PREFIX):
            literal_name = LiteralRegistry.LITERAL_PREFIX + literal_name
        return literal_name
        

class Literal:
    def __init__(self, literal_name_in_registry : str):
        self.literal_name_in_registry = literal_name_in_registry
    
    @abstractmethod
    def set(self, value : bool):
        pass

    @abstractmethod
    def __bool__(self):
        pass
    
    @abstractmethod
    def __str__(self):
        pass

    @abstractmethod
    def __repr__(self):
        pass

class SimpleLiteral(Literal):
    def __init__(self, literal_name_in_registry : str, value : bool):
        super().__init__(literal_name_in_registry)
        self.__value = value

    def set(self, value : bool):
        assert isinstance(value, bool)
        self.__value = value

    def __bool__(self):
        return self.__value

    def __str__(self):
        return self.literal_name_in_registry + "(Current value: "+str(bool(self))+")"

    def __repr__(self):
        return self.__str__()

class FunctionLiteral(Literal):
    def __init__(self, literal_name_in_registry : str, formula : Callable):
        
        super().__init__(literal_name_in_registry)

        self.__formula : Callable = formula

        self.literal_name_in_registry = literal_name_in_registry
        self.values_names_in_value_registry = self.get_parameter_names_from_formula(self.__formula)
    
    def set(self, formula : Callable):
        assert isinstance(formula, Callable)
        self.__formula = formula

    def get_parameter_names_from_formula(self, formula : Callable):
        parameter_names = []
        for name in inspect.getfullargspec(formula).args:
            if "=" in name:
                raise Exception("The Callable passed as formula should not contain default values")
            parameter_names.append(name)
        return parameter_names

    def __bool__(self):
        values_needed_by_formula = {}
        for value_name in self.values_names_in_value_registry:
            values_needed_by_formula[value_name] = ValueRegistry()[value_name]
        return self.__formula(**values_needed_by_formula)

    def __str__(self):
        return "%s(%s)(Current value: %s)" % (self.literal_name_in_registry, "\n".join(inspect.getsource(self.__formula).split("\n")[1:]).strip(), str(bool(self)))

    def __repr__(self):
        return self.__str__()

if __name__ == "__main__":
    print("Tests: ValueRegistry creation and print")
    value_registry = ValueRegistry()

    value_registry["distance"] = 100
    print(value_registry)

    ValueRegistry()["threshold1"] = 200
    ValueRegistry()["threshold2"] = 50
    print(ValueRegistry())
    print("\n\n")

    print("Tests: LiteralRegistry creation and print")
    literal_registry = LiteralRegistry()
    literal_registry["this_value_should_be_true"] = True
    print(literal_registry)

    LiteralRegistry()["this_value_should_be_false"] = False
    print(LiteralRegistry())
    print("\n\n")

    print("Tests: SimpleLiteral evaluation")
    print("this_value_should_be_true AND this_value_should_be_false SHOULD BE False: "+str(literal_registry["this_value_should_be_true"] and literal_registry["this_value_should_be_false"]))
    print("this_value_should_be_true OR this_value_should_be_false SHOULD BE True: "+str(literal_registry["this_value_should_be_true"] or literal_registry["this_value_should_be_false"]))
    print("\n\n")

    print("Tests: FunctionLiteral evaluation")
    def distance_lower_than_threshold1(distance, threshold1):
        return distance < threshold1

    def distance_lower_than_threshold2(distance, threshold2):
        return distance < threshold2
    
    LiteralRegistry().add_function_literal(distance_lower_than_threshold1)
    print("Evaluating: %s\nValue: %s\nShould be: %s" % (LiteralRegistry().get_literal_instance("distance_lower_than_threshold1"), LiteralRegistry()["distance_lower_than_threshold1"], "True"))
    LiteralRegistry()["distance_lower_than_threshold2"] = distance_lower_than_threshold2
    print("Evaluating: %s\nValue: %s\nShould be: %s" % (LiteralRegistry().get_literal_instance("distance_lower_than_threshold2"), LiteralRegistry()["distance_lower_than_threshold2"], "False"))
    print("\n\n")