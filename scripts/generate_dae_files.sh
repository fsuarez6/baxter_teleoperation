#!/bin/sh
#~ This script generates the .dae files for all the .urdf robots 
#~ found in the baxter_teleop/openrave

DESCRIPTION_PKG=`rospack find baxter_teleop`
FILES=$(find $DESCRIPTION_PKG/openrave -type f -name *.urdf)

for FILE in $FILES
do
  FILENAME=$(basename "$FILE")
  FILENAME="${FILENAME%.*}"
  URDF=$DESCRIPTION_PKG/openrave/$FILENAME.urdf
  DAE=$DESCRIPTION_PKG/openrave/$FILENAME.dae
  rosrun collada_urdf urdf_to_collada $URDF $DAE
  echo "URDF successfully generated for [$FILENAME.urdf]"
  #~ rosrun moveit_ikfast round_collada_numbers.py $DAE $DAE 5 > /dev/null
  openrave.py $DAE
done

