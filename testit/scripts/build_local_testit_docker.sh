#!/bin/bash

QUERY=$(docker images -q testitros/testit:base)
if [ ${#QUERY} -eq 0 ]; then
  echo "Building testitros/testit:base container..."
  cd $(rospack find testit)/docker
  docker build --no-cache -t testitros/testit:base .
fi
