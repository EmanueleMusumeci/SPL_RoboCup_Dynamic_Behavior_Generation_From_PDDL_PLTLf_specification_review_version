import random
import os
import copy
import argparse
from pathlib import Path


parser = argparse.ArgumentParser(description='Generates all possible combinations for scenes of challenge 1 and the bash script to launch them.')

parser.add_argument('--robocup_repository_root', 
                    type=str,
                    help='Root of the robocup repository (including "/spqrnao202*"')

parser.add_argument('--non_fast_scenes', 
                    action="store_true", default = False,
                    help='Whether to generate fast or non-fast scenes')

parser.add_argument('--obstacles_x_pos',
                    type=str, 
                    help='List of obstacle x positions separated by commas (e.g. X1,X2,X3)')

parser.add_argument('--obstacles_y_pos',
                    type=str, 
                    help='List of obstacle y positions separated by commas (e.g. Y1,Y2,Y3)')

args = parser.parse_args()

OBSTACLE_X_POS = [-850, -2700, -3500]
OBSTACLE_Y_POS = [0, 445, 345, -345]

#CHANGE THIS
ROOT_DIR = '/home/asc/robocup/spqrnao2021'
if args.robocup_repository_root:
    ROOT_DIR = args.robocup_repository_root
SCENE_DIR = os.path.join(ROOT_DIR, "Config", "Scenes")
HOME_DIR = str(Path.home())

if not os.path.exists(os.path.join(SCENE_DIR, "challenge1")):
    os.makedirs(os.path.join(SCENE_DIR, "challenge1"))

FAST = True
if args.non_fast_scenes:
    FAST = False

xPos = OBSTACLE_X_POS
if args.obstacles_x_pos:
    xPos = [int(xObs.lstrip().rsplit()) for xObs in args.obstacles_x_pos.split(",")]

yPos = OBSTACLE_Y_POS
if args.obstacles_y_pos:
    yPos = [int(yObs.lstrip().rsplit()) for yObs in args.obstacles_y_pos.split(",")]

sceneCounter = 0

def generateChallengeScene(sceneCount, couples):
    with open(os.path.join(SCENE_DIR, "challenge1", "challenge1scene"+str(sceneCount)+".ros2"), mode="w") as f:
        header = '<Simulation>\n\n\
<Include href="../Includes/NaoV6H25.rsi2"/>\n\
<Include href="../Includes/Ball2016SPL.rsi2"/>\n\
<Include href="../Includes/Field2017SPL.rsi2"/>\n\n\
<Scene name="RoboCup" controller="SimulatedNao" stepLength="0.012" color="rgb(65%, 65%, 70%)" ERP="0.8" CFM="0.001" contactSoftERP="0.2" contactSoftCFM="0.005">\n\
\t<Light z="9m" ambientColor="rgb(50%, 50%, 50%)"/>\n\n\
\t<Compound name="teamColors">\n\
\t\t<Appearance name="black"/>\n\
\t\t<Appearance name="blue"/>\n\
\t</Compound>\n\n'
        playing_robot = '\t<Compound name="robots">\n\
\t\t<Body ref="Nao" name="robot3">\n\
\t\t<Translation x="{}mm" y="{}mm" z="320mm"/>\n\
\t\t<Rotation z="{}degree"/> \n\
\t\t</Body>\n\
\t</Compound>\n\n'
        if FAST:
            playing_robot_pos = (850, 0, 180)
        else:
            playing_robot_pos = (850, -3000, 90)

        playing_robot = playing_robot.format(playing_robot_pos[0], playing_robot_pos[1], playing_robot_pos[2])

        non_playing_robots = '\t<Compound name="extras">\n\
\t\t<Body ref="NaoDummy" name="robot6">\n\
\t\t<Translation x="{}mm" y="{}mm" z="320mm"/>\n\
\t\t<Rotation z="0degree"/>\n\
\t\t<Set name="NaoColor" value="dblue"/>\n\
\t\t</Body>\n\
\t\t<Body ref="NaoDummy" name="robot7">\n\
\t\t<Translation x="{}mm" y="{}mm" z="320mm"/>\n\
\t\t<Rotation z="0degree"/>\n\
\t\t<Set name="NaoColor" value="dblue"/>\n\
\t\t</Body>\n\
\t\t<Body ref="NaoDummy" name="robot8">\n\
\t\t<Translation x="{}mm" y="{}mm" z="320mm"/>\n\
\t\t<Rotation z="0degree"/>\n\
\t\t<Set name="NaoColor" value="dblue"/>\n\
\t\t</Body>\n\
\t\t<Body ref="NaoDummy" name="robot9">\n\
\t\t<Translation x="{}mm" y="{}mm" z="320mm"/>\n\
\t\t<Rotation z="0degree"/>\n\
\t\t<Set name="NaoColor" value="dblue"/>\n\
\t\t</Body>\n\
\t</Compound>\n\n'.format(str(couples[0][0]), str(couples[0][1]), 
                        str(couples[1][0]), str(couples[1][1]), str(couples[1][0]), str(-couples[1][1]),
                        str(couples[2][0]), str(couples[2][1]))
        footer = '\t<Compound name="balls">\n\
\t\t<Body ref="ball">\n\
\t\t<Translation z="1m"/>\n\
\t\t</Body>\n\
\t</Compound>\n\n\
\t<Compound ref="field"/>\n\n\
</Scene>\n\n\
</Simulation>'

        f.write(header + playing_robot + non_playing_robots + footer)

    if FAST:
        launch_script = 'call Includes/Fast\n\n'
    else:
        launch_script = 'call Includes/Normal\n\n'
    
    launch_script+="dr debugDrawing3d:representation:OpponentGoalModel\n\
                    dr debugDrawing3d:module:BallPathProvider\n\
                    dr debugDrawing3d:representation:BallCarrierPFModel\n\
                    dr debugDrawing3d:representation:BallCarrierModel\n"
    
    #In non-fast scenes the robot has to start from the side of the field to localize
    if FAST:
        launch_script+= "gc playing"
    else:
        launch_script+= "gc ready"

    with open(os.path.join(SCENE_DIR, "challenge1", "challenge1scene"+str(sceneCount)+".con"), mode="w") as f:
        f.write(launch_script)

    print("Scene: challenge1scene"+str(sceneCount)+" created")

def generateBashScript():
    with open(os.path.join(SCENE_DIR, "challenge1", "startChallenge1.sh"), mode="w") as f:
        f.write('#!/bin/bash\n\n\
echo "Looking for files in directory $(dirname "$0")"\n\
dir="$( dirname "$0")"\n\n\
n_files=`/bin/ls -1 "$dir" | grep \'\.ros2$\' | wc -l | cut -f1`\n\n\
rand_num=`awk "BEGIN{srand();print int($n_files * rand()) + 1;}"`\n\n\
file=`/bin/ls -1 "$dir" | grep \'\.ros2$\' | sed -ne "${rand_num}p"`\n\
SCENE_PATH=`cd $dir && echo "$PWD/$file"`\n\
echo "Opening scene: $SCENE_PATH"\n\n\
#If the script is not working write below here the absolute path of the spqrnao202* directory (git repo root directory)\n\
REPO_ROOT="'+ROOT_DIR+'"\n\
echo "spqrnao git repo root is: $REPO_ROOT"\n\n\
cd $REPO_ROOT\n\n\
$REPO_ROOT/Build/Linux/SimRobot/Develop/SimRobot $SCENE_PATH')
     

def auxRecurse(sceneCount, xPositions, yPositions, couples):
    if len(xPositions) == 0:
        print(couples)
        generateChallengeScene(sceneCount, couples)
        return sceneCount + 1

    for x in xPositions:
        for y in yPositions:
            newCouples = copy.deepcopy(couples)
            newCouples.append((x,y))

            newXPositions = []
            for xVal in xPositions:
                if xVal == x: continue
                newXPositions.append(xVal)

            newYPositions = []
            for yVal in yPositions:
                if yVal == y: continue
                newYPositions.append(yVal)
            
            
            sceneCount = auxRecurse(sceneCount, newXPositions, newYPositions, newCouples)
    
    return sceneCount


couples = auxRecurse(0, xPos, yPos, [])
generateBashScript()