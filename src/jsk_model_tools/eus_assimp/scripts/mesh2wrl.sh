#!/bin/bash
SCALE=1.0
if [ $# -gt 2 ]; then
    SCALE=$3
fi

echo "convert $1 to $2 by eus-assimp with scale $SCALE"

roseus "(load \"package://eus_assimp/euslisp/eus-assimp.l\")" "(setq glv (load-mesh-file \"$1\" :scale $SCALE))" "(save-mesh-file \"$2\" glv)" "(exit)"
