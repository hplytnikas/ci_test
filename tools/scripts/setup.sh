#!/bin/bash

install_dir="/usr/local/share/amz"

export ONNXRUNTIME_DIR=$(find $install_dir -type d -name "onnxruntime*" | head -n 1)
export ACADOS_DIR=$(find $install_dir -type d -name "acados*" | head -n 1)
export PYLON_ROOT="/opt/pylon"
export EASYPROFILER_DIR="/usr/local/include/easy"

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ONNXRUNTIME_DIR/lib"  # for onnxruntime
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ACADOS_DIR/lib"       # for acados
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PYLON_ROOT/lib"       # for pylon
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"        # for easyprofiler
