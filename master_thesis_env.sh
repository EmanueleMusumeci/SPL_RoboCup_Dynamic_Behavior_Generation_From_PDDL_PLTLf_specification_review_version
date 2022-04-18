#!/bin/sh

#modify the right hand side with the root of your robocup folder
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
SPL_ROOT=$( cd "$(dirname "$parent_path")" ; pwd -P )

export SPLROOT=${SPL_ROOT}/spqrnao2021
export SPLCLIENTS=${SPL_ROOT}/spqrnao2021/external_clients
export SPLBUILD=${SPL_ROOT}/spqrnao2021/Build
export SPLCONFIG=${SPL_ROOT}/spqrnao2021/Config
export SPLSCENES=${SPL_ROOT}/spqrnao2021/Config/Scenes
export SPLSCENEBHF=${SPL_ROOT}/spqrnao2021/Config/Scenes/BHFast.ros2
export SPLSCENEHW5=${SPL_ROOT}/spqrnao2021/Config/Scenes/HomeWork5.ros2
export SPLMAKE=${SPL_ROOT}/spqrnao2021/Make/Linux
export SPLREP=${SPL_ROOT}/spqrnao2021/Src/Representations
export SPLMOD=${SPL_ROOT}/spqrnao2021/Src/Modules
export SPLSRC=${SPL_ROOT}/spqrnao2021/Src
export SPLSIM=${SPL_ROOT}/spqrnao2021/Build/Linux/SimRobot/Develop
export SPLCONTROLLER=${SPL_ROOT}/GameController
export SPLBUSH=${SPL_ROOT}/spqrnao2021/Build/Linux/bush/Develop
export SPLINSTALL=${SPL_ROOT}/spqrnao2021/Install


alias ltlmcd="cd $SPLMAKE; make CONFIG=Develop"

alias ltlsr="cd $SPLSIM; ./SimRobot"

alias ltlsrbhf="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/BHFast-devel.ros2"
alias ltlsr1v3d="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/1vs3Dummies.ros2"
alias ltlsr2v3d="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/2vs3Dummies.ros2"
alias ltl_fond_striker_no_obstacle="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/fond_striker_no_obstacle.ros2"
alias ltl_fond_striker_with_obstacle="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/fond_striker_with_obstacle.ros2"
alias ltl_fond_striker_jolly_no_obstacle="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/fond_striker_jolly_no_obstacle.ros2"
alias ltl_fond_striker_jolly_with_obstacle="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/fond_striker_jolly_with_obstacle.ros2"
alias ltlsrotf="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/OneTeamFast.ros2"
alias ltlsrgfd="reset; cd $SPLSIM; ./SimRobot $SPLSCENES/GameFast-devel.ros2"

alias srrr="cd $SPLSIM; ./SimRobot $SPLSCENES/RemoteRobot.ros2"

alias ltlcopd="cd $SPLMAKE; ./copyfiles Develop -b -v 60 "
alias ltlcopdnorestart="cd $SPLMAKE; ./copyfiles Develop -v 60 "

#alias ltlpc="/home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run.py"

alias ltldfa0="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.reach_ball_and_kick"
alias ltldfa1="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.reach_ball_and_kick_until_goal"
alias ltldfa2="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.patrolling_with_static_waypoints_without_mutual_exclusion"
alias ltldfa3="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.patrolling_with_static_waypoints_without_mutual_exclusion_stateless"
alias ltldfa4="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.reach_waypoint_then_reach_ball_and_kick_until_goal"
alias ltldfa5="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py SPL_examples.patrolling_with_static_waypoints_with_mutual_exclusion_stateful"

alias ltlcpbs="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py classical_planning_examples.basic_striker"
alias ltlcpbsl="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py classical_planning_examples.basic_striker --localhost"
alias ltlcpsspl="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py classical_planning_examples.striker_supporter_pass --localhost"

alias ltlre="reset; /home/asc/anaconda3/envs/robocup/bin/python3 $SPLCLIENTS/run_experiment.py "

alias ltlnc="node $SPLCLIENTS/web_interface/clientUDP.js"

alias ltlrtm="bash $SPLROOT/run_tmux.sh"
