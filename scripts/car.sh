#!/bin/bash

#this script will be used to run foxglove websocket

ssh -L 9091:localhost:9091 -E /dev/null nvidia@192.168.90.231
