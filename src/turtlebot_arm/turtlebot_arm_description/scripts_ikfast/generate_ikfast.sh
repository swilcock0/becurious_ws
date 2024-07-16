#!/bin/bash

#set -e #fail on error
. ~/becurious_ws/devel/setup.bash

export MYROBOT_NAME="pincher"
export PLANNING_GROUP="arm"
roscd turtlebot_arm_description/urdf
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME"_"$PLANNING_GROUP".urdf.xacro
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae

export IKFAST_PRECISION="5"
cp "$MYROBOT_NAME".dae "$MYROBOT_NAME".backup.dae  # create a backup of your full precision dae.
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"

openrave-robot.py "$MYROBOT_NAME".dae --info links
export BASE_LINK="1"
export EEF_LINK="23"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp

cat <<EOF > ./$MYROBOT_NAME.xml
<robot file="$MYROBOT_NAME.dae">
  <Manipulator name="$MYROBOT_NAME">
    <base>arm_base_link</base>
    <effector>gripper_link</effector>
  </Manipulator>
</robot>
EOF

openrave.py --database inversekinematics --robot=$MYROBOT_NAME.xml --iktype=TranslationDirection5D --iktests=500

export PLUGIN_PKG="$MYROBOT_NAME""_ikfast_""$PLANNING_GROUP""_plugin"

roscd turtlebot_arm_description/..
catkin_create_pkg "$PLUGIN_PKG"

cd ..
catkin_make
. ~/becurious_ws/devel/setup.bash

roscd turtlebot_arm_description
python ./scripts_ikfast/create_ikfast.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$PLUGIN_PKG" $(find /home/ros/.openrave | grep cpp --color=never)

cd ~/becurious_ws

catkin_make