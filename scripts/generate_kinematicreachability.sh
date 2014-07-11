#!/bin/bash
FILENAME=$(basename "$0")
usage()
{
cat << EOF
usage: rosrun baxter_teleop $FILENAME [options]

This script generates the kinematic reachability for the grips.

OPTIONS:
   -h     Show this message
   -d     Generate .dae files before
   -s     Only shows the previous calculated kinematic reachability
   -v     Verbose
EOF
}

SHOW=0
DAE=0
while getopts hsd opts; do
  case ${opts} in
    h)
      usage
      exit 1 ;;
    s) 
      SHOW=1 ;;
    g) 
      DAE=1 ;;
  esac
done

if [ $DAE = 1 ]; then
  rosrun baxter_teleop generate_dae_files.sh
fi
# Grips
DESCRIPTION_PKG=`rospack find baxter_teleop`
ROBOT_XML=$DESCRIPTION_PKG/openrave/baxter.robot.xml
# 6D Transform
if [ $SHOW = 0 ]; then
  MSG_KIN_REACH="Generating [kinematicreachability] from: $LEFT_XML"
  echo $MSG_KIN_REACH
  openrave.py --database kinematicreachability --robot=$ROBOT_XML --manipname=left_arm --xyzdelta=0.02 --numthreads=8
  openrave.py --database kinematicreachability --robot=$ROBOT_XML --manipname=right_arm --xyzdelta=0.02 --numthreads=8
  #~ RESULTS FOR ACCUARATE ARMS
  #~ xyzdelta=0.04
  #~ openravepy.databases.kinematicreachability: generatepcg, radius: 1.379102, xyzsamples: 171294, quatdelta: 0.513496, rot samples: 72, freespace: 0
  #~ openravepy.databases: generate, database ReachabilityModel finished in 149686.343989s
  #~ xyzdelta=0.02
  #~ openravepy.databases.kinematicreachability: generatepcg, radius: 1.342729, xyzsamples: 1267760, quatdelta: 0.513496, rot samples: 72, freespace: 0
  #~ xyzdelta=0.01
  #~ openravepy.databases.kinematicreachability: generatepcg, radius: 1.319542, xyzsamples: 9621204, quatdelta: 0.513496, rot samples: 72, freespace: 0
  
  #~ RESULTS FOR HOLE BAXTER
  #~ xyzdelta=0.04
  #~ openravepy.databases.kinematicreachability: generatepcg, radius: 1.374097, xyzsamples: 169590, quatdelta: 0.513496, rot samples: 72, freespace: 0
  #~ xyzdelta=0.02
  #~ 
  #~ xyzdelta=0.01
  #~ 
fi
# Show
MSG="Showing [kinematicreachability] for: $ROBOT_XML"
echo $MSG
openrave.py --database kinematicreachability --robot=$ROBOT_XML --show --showscale=1.285714286
