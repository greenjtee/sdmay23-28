#!/bin/bash

# create dependencies directory if it doesn't exist
if [ ! -d "dependencies" ]; then
    mkdir dependencies
fi

export OPENLANE_ROOT=$(pwd)/dependencies/openlane_src # you need to export this whenever you start a new shell

export PDK_ROOT=$(pwd)/dependencies/pdks # you need to export this whenever you start a new shell

# export the PDK variant depending on your shuttle, if you don't know leave it to the default

# for sky130 MPW shuttles....
export PDK=sky130A
