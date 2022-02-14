#!/bin/bash

echo "Launching Python server..."; 
trap "echo 'Killing Python server (PID: $PYTHON_SERVER_PID)'; kill -9 $PYTHON_SERVER_PID" SIGINT SIGKILL
python $LTLCLIENTS/async_socket_NAO.py &
PYTHON_SERVER_PID=$!
echo "PYTHON_SERVER_PID: $!"
wait $PID
trap - TERM INT