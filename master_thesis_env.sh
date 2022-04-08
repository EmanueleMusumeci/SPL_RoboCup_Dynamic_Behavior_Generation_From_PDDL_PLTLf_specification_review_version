#!/bin/sh

#modify the right hand side with the root of your robocup folder
LTL_ROOT=~/ltl_behavior_synthesis

export LTLROOT=${LTL_ROOT}/spqrnao2021
export LTLCLIENTS=${LTL_ROOT}/spqrnao2021/external_clients
export LTLBUILD=${LTL_ROOT}/spqrnao2021/Build
export LTLCONFIG=${LTL_ROOT}/spqrnao2021/Config
export LTLSCENES=${LTL_ROOT}/spqrnao2021/Config/Scenes
export LTLSCENEBHF=${LTL_ROOT}/spqrnao2021/Config/Scenes/BHFast.ros2
export LTLSCENEHW5=${LTL_ROOT}/spqrnao2021/Config/Scenes/HomeWork5.ros2
export LTLMAKE=${LTL_ROOT}/spqrnao2021/Make/Linux
export LTLREP=${LTL_ROOT}/spqrnao2021/Src/Representations
export LTLMOD=${LTL_ROOT}/spqrnao2021/Src/Modules
export LTLSRC=${LTL_ROOT}/spqrnao2021/Src
export LTLSIM=${LTL_ROOT}/spqrnao2021/Build/Linux/SimRobot/Develop
export LTLCONTROLLER=${LTL_ROOT}/GameController
export LTLBUSH=${LTL_ROOT}/spqrnao2021/Build/Linux/bush/Develop
export LTLINSTALL=${LTL_ROOT}/spqrnao2021/Install


alias ltlmcd="cd $LTLMAKE; make CONFIG=Develop"

alias ltlsr="cd $LTLSIM; ./SimRobot"

alias ltlsrbhf="cd $LTLSIM; ./SimRobot $LTLSCENES/BHFast-devel.ros2"
alias ltlsr1v3d="cd $LTLSIM; ./SimRobot $LTLSCENES/1vs3Dummies.ros2"
alias ltlsr2v3d="cd $LTLSIM; ./SimRobot $LTLSCENES/2vs3Dummies.ros2"
alias ltlsrotf="cd $LTLSIM; ./SimRobot $LTLSCENES/OneTeamFast.ros2"
alias ltlsrgfd="cd $LTLSIM; ./SimRobot $LTLSCENES/GameFast-devel.ros2"

alias srrr="cd $LTLSIM; ./SimRobot $LTLSCENES/RemoteRobot.ros2"

alias ltlcopd="cd $LTLMAKE; ./copyfiles Develop -b -v 60 "
alias ltlcopdnorestart="cd $LTLMAKE; ./copyfiles Develop -v 60 "

#alias ltlpc="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run.py"

alias ltldfa0="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.reach_ball_and_kick"
alias ltldfa1="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.reach_ball_and_kick_until_goal"
alias ltldfa2="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.patrolling_with_static_waypoints_without_mutual_exclusion"
alias ltldfa3="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.patrolling_with_static_waypoints_without_mutual_exclusion_stateless"
alias ltldfa4="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.reach_waypoint_then_reach_ball_and_kick_until_goal"
alias ltldfa5="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py LTL_examples.patrolling_with_static_waypoints_with_mutual_exclusion_stateful"

alias ltlcpbs="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py classical_planning_examples.basic_striker"
alias ltlcpssp="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_experiment.py classical_planning_examples.striker_supporter_pass"

alias ltlnc="node $LTLCLIENTS/web_interface/clientUDP.js"

alias ltlrtm="bash $LTLROOT/run_tmux.sh"
