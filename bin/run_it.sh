#!/bin/bash

#The old indian trick to switch to the bin/ directory across different workspaces/systems/installations
#Works every time..

STARTDIR=`pwd` 
#Force switch to this directory ( ROS Fuerte seems to go to ~/.ros for some reason :P )
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
cd ..
cd ..
ROOT_ROS_WORKSPACE=`pwd`
cd "$DIR"

red=$(printf "\033[31m")
green=$(printf "\033[32m") 
normal=$(printf "\033[m")

export GLOG_logtostderr=1
 
echo `pwd`

WORKNAME="whole_body_ik"

#Bin name has actually a _node postfix
BINNAME="whole_body_ik_node"

PATHOFBIN="$ROOT_ROS_WORKSPACE/devel/lib/$WORKNAME/$BINNAME"
if [ -e $PATHOFBIN ]
then
 echo "$green Running freshly compiled file $normal" 
else 
 echo "$red Could not find freshly compiled file @ $PATHOFBIN $normal" 
 PATHOFBIN="./$BINNAME" 
  
 if [ ! -e $PATHOFBIN ]
   then 
     echo "$red Could not find $BINNAME binary!  $normal"
   else
     echo "$green Found $BINNAME binary! $normal"
   fi
fi 

LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH "$PATHOFBIN" $@

cd "$STARTDIR"
exit 0
