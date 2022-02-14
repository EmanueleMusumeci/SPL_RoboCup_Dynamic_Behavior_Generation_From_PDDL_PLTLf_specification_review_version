#!/bin/bash

kill_tmux()
{
    echo "Killing tmux session... "
    . $SCRIPT_DIR/kill_process_if_exists.sh tmux
    echo "DONE"
}

check_package_installed()
{
    dpkg -s $name &> /dev/null  

    return test $? -ne 0
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#set -e

bash $SCRIPT_DIR/master_thesis_env.sh


if [ $# -eq 0 ]
  then
    SELECTED_SCENE=$LTLSCENES/1vs3Dummies.ros2
    echo "No scene specified, defaulting to $SELECTED_SCENE"
else
    SELECTED_SCENE=$LTLSCENES/1vs3Dummies.ros2
    if [[ ! -f "$SELECTED_SCENE" ]]; then
        echo "$SELECTED_SCENE  does not exist."
        exit 0
    fi
    echo "Selected scene $SELECTED_SCENE"
fi


if pip list | grep -F "twisted"; then
    echo "Installing twisted with pip package manager... "
    pip install twisted
    echo "DONE"
fi

if pip list | grep -F "service_identity"; then
    echo "Installing service_identity with pip package manager... "
    pip install service_identity
    echo "DONE"
fi

if pip list | grep -F "service_identity"; then
    echo "Installing service_identity with pip package manager... "
    pip install service_identity
    echo "DONE"
fi

if ! dpkg -s tmux; then
    echo "Please install tmux package!!!"
    exit 0
else
    echo "tmux installation found but no custom configuration file: applying configuration."
    if [[ -f "~/tmux.conf" ]]; then
        echo "Do you wish to apply the custom configuration (contained in tmux.conf)? This will overwrite the existing configuration."
    else
        echo "Do you wish to apply the custom configuration (contained in tmux.conf)?"
    fi
    select yn in "Yes" "No"; do
        case $yn in
            Yes ) cp $SCRIPT_DIR/tmux.conf ~/.tmux.conf; break;;
            No ) break;;
        esac
    done
fi


trap kill_tmux SIGINT
trap kill_tmux SIGKILL
trap kill_tmux SIGQUIT
trap kill_tmux SIGHUP
trap kill_tmux SIGTERM
trap kill_tmux EXIT

GUI_PORT="3000"
GUI_IP="127.0.0.1"
GUI_ADDRESS="$GUI_IP:$GUI_PORT"


#Check if tmux session named "RoboCup" exists and terminate it
if tmux has-session -t "RoboCup" 2>/dev/null; then
  # Kill existing session
  tmux kill-session -t "RoboCup"
fi

#Create tmux session
tmux new-session -s "RoboCup" -n "RoboCup SPL LTL-based Dynamic Behavior Generation"  \; split-window -h \; split-window -v \; \
    select-pane -t 0 \; \
    send-keys "bash $SCRIPT_DIR/launch_simrobot.sh $SELECTED_SCENE" C-m \; \
    select-pane -t 1 \; \
    send-keys "bash $SCRIPT_DIR/launch_python_server.sh" C-m \; \
    select-pane -t 2 \; \
    send-keys "bash $SCRIPT_DIR/launch_nodejs_server.sh $GUI_ADDRESS" C-m 
TMUX_PID=$!

if [[ -z $(google-chrome $GUI_ADDRESS) ]]; then
    firefox "$GUI_ADDRESS"
fi

wait