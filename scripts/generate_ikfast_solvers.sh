#!/bin/sh
#~ Baxter
DESCRIPTION_PKG=`rospack find baxter_teleop`
DAE=$DESCRIPTION_PKG/openrave/baxter.dae
ROBOT_XML=$DESCRIPTION_PKG/openrave/baxter.robot.xml
#~ transformation6d
MSG_IKFAST_6D="Generating [transformation6d] ikfast plugin from: $DAE"
echo $MSG_IKFAST_6D
#~ python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=$DAE --iktype=transform6d --baselink=1 --eelink=7 --savefile=$CPP_IKFAST
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=left_arm --precision=5
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=right_arm --precision=5
echo "Finished ikfast generation"
#~ iktest
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=left_arm --usecached --iktests=1000 
