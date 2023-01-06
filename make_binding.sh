#!/bin/bash

# Uses https://github.com/RosettaCommons/binder to create pybind11 bindings

# Set the following env variables before invoking this script if you want something different
# than the defaults listed below
ROOT_MODULE=${ROOT_MODULE:="ActuatorController"}
PREFIX=${PREFIX:=$(pwd)}
BIND_DIR=${BIND_DIR:=binding}
INCLUDES=${INCLUDES:=all_includes.hpp}

mkdir -p "${PREFIX}/${BIND_DIR}"
binder --root-module ${ROOT_MODULE} --prefix ${PREFIX} --config ${PREFIX}/binder.config ${INCLUDES} 

#binder --root-module ActuatorController --prefix ~/wip/clover/Education_robot_py/ActuatorController_SDK/sdk/include --bind Actuator pybinding.hpp -- -I$HOME/wip/clover/Education_robot_py/ActuatorController_SDK/sdk/include -I$HOME/wip/clover/Education_robot_py/ActuatorController_SDK/sdk/lib/linux_x86_64/libActuatorController.so  -I/usr/include/c++/10 -I/usr/include/x86_64-linux-gnu/c++/10 -std=c++11
