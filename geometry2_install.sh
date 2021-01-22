#!/bin/bash

# find python3
PYTHON3_EXEC="$(which python3)"
PYTHON3_INCLUDE="$(ls -d /usr/include/* | grep  python | sort -r| head -1)"
PYTHON3_DLIB="$(ls -d /usr/lib/x86_64-linux-gnu/* | grep -P  "libpython3\S*.so"| sort | head -1)"
if [ -z $PYTHON3_DLIB ] || [ -z $PYTHON3_INCLUDE ] || [ -z $PYTHON3_EXEC ] ; then
    echo "Can't find python library please install it with \" sudo apt-get python3-dev \" !" >&2
fi

PARENT_DIR_WS="$(cd "$(dirname $0)/../../.." >/dev/null 2>&1 && pwd)"
mkdir -p ${PARENT_DIR_WS}/geometry2_ws/src && cd "$_"
git clone --depth=1 https://github.com/ros/geometry2.git
cd ..

# compile geometry2 with python3 
echo -n "compiling geometry2 with python3 ..."
catkin_make -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} -DPYTHON_INCLUDE_DIR=${PYTHON3_INCLUDE} -DPYTHON_LIBRARY=${PYTHON3_DLIB} > /dev/null 2>&1


# add the lib path to the python environment 
if [ $? -eq 0 ] ; then
    echo " done!"
    package_path="$(cd devel/lib/python3/dist-packages && pwd)"
    rc_info="export PYTHONPATH=${package_path}:\${PYTHONPATH}\n"
    if echo $SHELL | grep zsh > /dev/null
    then
        echo -e "$rc_info" >> ~/.zshrc
        echo "PYTHONPATH has been updated in your zshrc file."
    elif echo $SHELL | grep bash > /dev/null
    then
        echo -e "$rc_info" >> ~/.bashrc
        echo "PYTHONPATH has been updated in your bashrc file."
    else
        echo "Can't not determin which terminal you are using. Please manualy add the package path ${package_path} to you bashrc or zshrc file later"
    fi
else
    echo "Fail to compile geometry2"
fi
