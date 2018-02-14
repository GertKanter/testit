#!/bin/bash

cd $(rospack find testit)/docker
docker build --no-cache -t testitros/testit:base .
