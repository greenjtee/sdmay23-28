#!/bin/sh

# creates directory for dependencies, sets environment variables, and runs make install for openlane

mkdir dependencies

export OPENLANE_ROOT=$(pwd)/dependencies/openlane_src # you need to export this whenever you start a new shell

export PDK_ROOT=$(pwd)/dependencies/pdks # you need to export this whenever you start a new shell

# export the PDK variant depending on your shuttle, if you don't know leave it to the default
export PDK=sky130B

make setup
