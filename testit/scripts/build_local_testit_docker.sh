#!/bin/bash

QUERY=$(docker images -q testitros/testit:latest)
if [ ${#QUERY} -eq 0 ]; then
  echo "Building testitros/testit:latest container..."
  cd $(rospack find testit)/docker
  docker build --no-cache -t testitros/testit:latest .
fi
