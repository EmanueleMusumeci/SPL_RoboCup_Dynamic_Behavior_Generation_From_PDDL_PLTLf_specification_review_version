#!/bin/bash

echo "Terminating NodeJS server... (PID: $1))"; 
. ./kill_process_if_exists.sh $1
echo "DONE"
exit 0