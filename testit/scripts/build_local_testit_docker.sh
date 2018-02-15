#!/bin/bash

echo "Building testitros/testit:base container..."
cd $(rospack find testit)/docker
docker build --no-cache -t testitros/testit:base .
