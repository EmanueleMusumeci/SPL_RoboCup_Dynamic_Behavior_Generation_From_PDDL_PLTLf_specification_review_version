#!/bin/bash

echo "Launching NodeJS server..."; 
trap "echo 'Killing NodeJS server (PID: $NODE_SERVER_PID)'; kill -9 $NODE_SERVER_PID" SIGINT SIGKILL
node $LTLCLIENTS/web_interface/clientUDP.js &
NODE_SERVER_PID=$!
echo "NODE_SERVER_PID: $!"
wait $PID
trap - TERM INT