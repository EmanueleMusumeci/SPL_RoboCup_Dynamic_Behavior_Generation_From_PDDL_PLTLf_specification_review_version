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
alias ltlsrotf="cd $LTLSIM; ./SimRobot $LTLSCENES/OneTeamFast.ros2"
alias ltlsrgfd="cd $LTLSIM; ./SimRobot $LTLSCENES/GameFast-devel.ros2"

alias srrr="cd $RSIM; ./SimRobot $RSCENES/RemoteRobot.ros2"

alias copd="cd $RMAKE; ./copyfiles Develop -b -v 60 "
alias copdnorestart="cd $RMAKE; ./copyfiles Develop -v 60 "

alias ltlpc="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run.py"
alias ltldfa0="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_with_dfa_experiment_test.py"
alias ltldfa1="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_with_dfa_experiment_patrolling_with_static_params.py"
alias ltldfa2="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_with_dfa_experiment_patrolling_with_dynamic_params.py"
alias ltldfa3="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_with_dfa_experiment_basic_striker.py"
alias ltldfa4="/home/asc/anaconda3/envs/robocup/bin/python3 $LTLCLIENTS/run_with_dfa_experiment_repeated_passes.py"
alias ltlnc="node $LTLCLIENTS/web_interface/clientUDP.js"

alias ltlrtm="bash $LTLROOT/run_tmux.sh"
