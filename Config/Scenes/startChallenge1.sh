#!/bin/bash

echo "Looking for files in directory $(dirname "$0")"
dir="$( dirname "$0")"

n_files=`/bin/ls -1 "$dir" | grep '\.ros2$' | wc -l | cut -f1`

rand_num=`awk "BEGIN{srand();print int($n_files * rand()) + 1;}"`

file=`/bin/ls -1 "$dir" | grep '\.ros2$' | sed -ne "$rand_nump"`
SCENE_PATH=`cd $dir && echo "$PWD/$file"`
echo "Opening scene: $SCENE_PATH"

#If the script is not working write below here the absolute path of the spqrnao202* directory (git repo root directory)
REPO_ROOT="/home/asc/robocup/spqrnao2021"
echo "spqrnao git repo root is: $REPO_ROOT"

cd $REPO_ROOT

$REPO_ROOT/Build/Linux/SimRobot/Develop/SimRobot $SCENE_PATH