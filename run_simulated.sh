#!/bin/bash

kill_terminal()
{
    echo "Terminating SimRobot (PID: $1)"; 
    ps -e | grep -F "$1"
    if ps -e | grep -F "$1"; then
        kill $1
    fi
    echo "DONE"
    exit 0
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
bash $SCRIPT_DIR/master_thesis_env.sh

#Run SimRobot
echo "Run SimRobot... "
gnome-terminal --tab --title="SimRobot" -- bash -c "$LTLSIM/SimRobot $LTLSCENES/1vs3Dummies.ros2 || exit 1" &
TERMINAL_PID=$!
echo "DONE"

echo "Run servers... "
#until $(. $SCRIPT_DIR/run_nodes.sh | tee log.txt); do
#    echo "Network nodes crashed. Restarting... "
#    sleep 1
#done
. $SCRIPT_DIR/run_nodes.sh | tee log.txt
echo "DONE"

trap kill_terminal INT EXIT

wait
