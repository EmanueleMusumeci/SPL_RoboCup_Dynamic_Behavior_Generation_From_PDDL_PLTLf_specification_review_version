#!/bin/bash

if ps -e | grep -F "$1"; then
    ps -aux | grep -F "$1" | awk '{print $2}' | xargs kill -9
fi