#!/bin/bash

START_DIR=$PWD
LIBDAISY_DIR=$PWD/lib/libDaisy
DAISYSP_DIR=$PWD/lib/DaisySP

echo "building libDaisy . . ."
cd "$LIBDAISY_DIR" ; make -s clean ; make -j -s
if [ $? -ne 0 ]
then
    echo "Failed to compile libDaisy"
    exit 1
fi
echo "done."

echo "building DaisySP . . ."
cd "$DAISYSP_DIR" ; make -s clean ; make -j -s
if [ $? -ne 0 ]
then
    echo "Failed to compile DaisySP"
    exit 1
fi
echo "done."