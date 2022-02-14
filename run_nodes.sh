#!/bin/bash

kill_python_server()
{
    echo "Terminating Python server... (PID: $1)"; 
    ps -e | grep -F "$1"
    if ps -e | grep -F "$1"; then
        kill $1
    fi
    echo "DONE"
    exit 0
}

kill_node_server()
{
    echo "Terminating NodeJS server... (PID: $1))"; 
    if ps -e | grep -F "$1"; then
        kill $1
    fi
    echo "DONE"
    exit 0
}

kill_servers()
{
    echo "Killing server processes... "
    kill_python_server $PYTHON_SERVER_PID
    kill_node_server $NODE_SERVER_PID
    exit 0
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

set -ne

bash $SCRIPT_DIR/master_thesis_env.sh

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


trap kill_servers INT EXIT

GUI_PORT="3000"
GUI_IP="127.0.0.1"
GUI_ADDRESS="$GUI_IP:$GUI_PORT"

{ 
    echo "Launching Python server"; 
    {
        python $LTLCLIENTS/async_socket_NAO.py &
        NODE_SERVER_PID=$!
        echo "NODE_SERVER_PID: $NODE_SERVER_PID"
    } || kill_node_server $NODE_SERVER_PID
} &
NODE_SERVER_PID=$!

{
    echo "Launching NodeJS server"; 
    {
        node $LTLCLIENTS/web_interface/clientUDP.js &
        PYTHON_SERVER_PID=$!
        echo "PYTHON_SERVER_PID: $PYTHON_SERVER_PID"
    } || kill_python_server $PYTHON_SERVER_PID
} &
PYTHON_SERVER_PID=$!

if [[ -z $(google-chrome $GUI_ADDRESS) ]]; then
    firefox "$GUI_ADDRESS"
fi

wait