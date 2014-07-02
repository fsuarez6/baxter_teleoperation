#!/bin/sh
#~ Baxter
DESCRIPTION_PKG=`rospack find baxter_teleop`
LEFT_XML=$DESCRIPTION_PKG/openrave/baxter_arm.left.robot.xml
RIGHT_XML=$DESCRIPTION_PKG/openrave/baxter_arm.right.robot.xml
#~ transformation6d
MSG_IKFAST_6D="Generating [transformation6d] ikfast database from: $LEFT_XML"
echo $MSG_IKFAST_6D
openrave.py --database inversekinematics --robot=$LEFT_XML --manipname=left_arm --freejoint=left_w1

#~ TODO: baxter_arm.right.robot.xml has the same hash as baxter_arm.left.robot.xml. No idea why!
#~ openrave.py --database inversekinematics --robot=$RIGHT_XML --manipname=right_arm --freejoint=right_w1

#~ TODO: This iktest is not working. No idea why!
#~ openrave.py --database inversekinematics --robot=$LEFT_XML --manipname=left_arm --usecached --iktests=1000 
