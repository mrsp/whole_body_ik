#!/bin/bash


#The old indian trick to switch to the bin/ directory across different workspaces/systems/installations
#Works every time..

#roslaunch whole_body_ik nao_wbc.launch


STARTDIR=`pwd`

#Switch to this directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"


cd bin
./run_it.sh $@
cd ..

cd "$STARTDIR"

exit 0
