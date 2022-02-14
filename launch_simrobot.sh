#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
bash $SCRIPT_DIR/master_thesis_env.sh


if [ $# -eq 0 ]
  then
    echo "No scene specified. Exiting..."
    exit 1
fi

#Run SimRobot
echo "Run SimRobot... "
$LTLSIM/SimRobot $1 || exit 1
echo "DONE"