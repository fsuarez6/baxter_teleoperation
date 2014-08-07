#!/bin/sh
#~ Baxter
ROS_PKG=`rospack find baxter_openrave`
ROBOT_XML=$ROS_PKG/openrave/baxter.robot.xml
#~ transformation6d
MSG_IKFAST_6D="Generating [transformation6d] ikfast databases from: $ROBOT_XML"
echo $MSG_IKFAST_6D
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=left_arm --freejoint=left_w1
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=right_arm --freejoint=right_w1

#~ TODO: This iktest is not working. No idea why!
#~ openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=left_arm --usecached --iktests=1000
#~ openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=right_arm --usecached --iktests=1000
