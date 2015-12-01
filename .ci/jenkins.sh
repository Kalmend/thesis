#!/bin/bash
rm -rf deps
rm -rf results
mv .ci/* .
mkdir results 
if [ ${REBUILD_DEPS} -ne 0 ]; then nocache="--no-cache"; fi
docker build $nocache -t ros-sprec-deps deps/
docker build --no-cache -t ros-sprec .
docker run -v "${WORKSPACE}/results:/results" ros-sprec
