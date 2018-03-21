#!/bin/bash

echo "Building testit_tb_sut container..."
cd $(rospack find testit_tutorials)/tutorials/turtlebot/docker/sut
docker build --no-cache -t testit_tb_sut .
echo "Building testit_tb_testit container..."
cd $(rospack find testit_tutorials)/tutorials/turtlebot/docker/testit
docker build --no-cache -t testit_tb_testit .
