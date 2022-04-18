import math 
import difflib

def find_similar_strings(string, list_of_strings):
    return difflib.get_close_matches(string, list_of_strings)

def tab_all_lines_in_string(string, times = 1):
    result_string = "\t"
    
    lines  = string.split("\n")
    for i, line in enumerate(lines):
        if not line:
            result_string += ("\n" if i < len(lines) else "")
        else:
            result_string += line + ("\n" + "".join(["\t"] * times) if i < len(lines) else "")
    return result_string


#Add function that computes distance of supporter from striker to ValueRegistry
def linear_distance(point1, point2):
    assert isinstance(point1, tuple)
    assert len(point1) == 2 or len(point1) == 3

    if len(point1) == 2:
        point1_x = point1[0]
        point1_y = point1[1]
    else:
        point1_x = point1[1]
        point1_y = point1[2]

    assert isinstance(point1_x, float) or isinstance(point1_x, int)
    assert isinstance(point1_y, float) or isinstance(point1_y, int)



    assert isinstance(point2, tuple)
    assert len(point2) == 2 or len(point2) == 3

    if len(point2) == 2:
        point2_x = point2[0]
        point2_y = point2[1]
    else:
        point2_x = point2[1]
        point2_y = point2[2]

    assert isinstance(point2_x, float) or isinstance(point2_x, int)
    assert isinstance(point2_y, float) or isinstance(point2_y, int)

    return math.sqrt(pow(point1_x-point2_x, 2) + pow(point1_y-point2_y, 2))

def angular_distance(point1, point2):
    assert isinstance(point1, tuple)
    assert len(point1) == 2 or len(point1) == 3

    if len(point1) == 2:
        point1_angle = None
    else:
        point1_angle = point1[0]
        assert isinstance(point1_angle, float)


    assert isinstance(point2, tuple)
    assert len(point2) == 2 or len(point2) == 3

    if len(point2) == 2:
        point2_angle = None
    else:
        point2_angle = point2[0]
        assert isinstance(point2_angle, float)

    if (point1_angle is None) or (point2_angle is None):
        return 0
    else:
        return abs(point1_angle - point2_angle)

def angular_distance_in_degrees(point1, point2):
    return math.degrees(angular_distance(point1, point2))