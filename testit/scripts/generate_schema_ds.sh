#!/bin/bash

# Install: pip install generateDS
XSD=junit-10.xsd
TESTIT_DIR=$(rospack find testit)
generateDS -f -o $TESTIT_DIR/src/testit/junit.py $TESTIT_DIR/xsd/$XSD
