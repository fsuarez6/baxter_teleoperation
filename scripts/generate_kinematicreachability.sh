#!/bin/bash
FILENAME=$(basename "$0")
usage()
{
cat << EOF
usage: rosrun grips_kinematics $FILENAME [options]

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
  rosrun grips_description generate_dae_files.sh
fi
# Grips
DESCRIPTION_PKG=`rospack find baxter_teleop`
DAE=$DESCRIPTION_PKG/openrave/baxter.dae
ROBOT_XML=$DESCRIPTION_PKG/openrave/baxter.robot.xml
# 6D Transform
if [ $SHOW = 0 ]; then
  MSG_KIN_REACH="Generating [kinematicreachability] from: $DAE"
  echo $MSG_KIN_REACH
  openrave.py --database kinematicreachability --robot=$ROBOT_XML --manipname=left_arm --xyzdelta=0.02 --numthreads=8
fi
# Show
MSG="Showing [kinematicreachability] for: $DAE"
echo $MSG
openrave.py --database kinematicreachability --robot=$ROBOT_XML --show --showscale=8
